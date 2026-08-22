#!/usr/bin/env python3
"""onshape_fetch.py - pull FRC build data (gear ratios, dimensions, BOM, mass
properties) from an Onshape document via the REST API.

Single file, Python 3 standard library ONLY (no pip installs). Emits ONE JSON
object to stdout; diagnostics go to stderr. Designed to be invoked by the
`frc-robot-coder` Claude skill (see ../onshape-integration.md).

Design rules (mirrors the skill doctrine):
  * The script returns RAW API data + CANDIDATE parses + a confidence flag.
    Claude does the real reasoning (COTS resolution, parallel-axis decisions,
    provenance marking). Keep heavy logic out of here.
  * Credentials come from env vars ONSHAPE_ACCESS_KEY / ONSHAPE_SECRET_KEY
    (or a gitignored local file). The secret is NEVER printed, logged, or put
    into the JSON output / exceptions.
  * Onshape has a small annual API quota (~2,500 calls/yr; only 2xx/3xx count).
    So: cache to disk, prefer immutable v/ version reads, short-circuit the
    ratio cascade, back off on 429, stop on 402.

Auth: HTTP Basic (access key = username, secret key = password) over HTTPS.
Base URL: https://cad.onshape.com/api/v10
"""

import argparse
import base64
import hashlib
import json
import os
import re
import sys
import time
import unicodedata
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

DEFAULT_BASE_URL = "https://cad.onshape.com/api/v10"
ACCEPT_HEADER = "application/json;charset=UTF-8;qs=0.09"
CONFIDENCE_ORDER = {"high": 0, "medium": 1, "low": 2}

# Keyword sets used to classify Onshape variables.
RATIO_NAME_HINTS = ("ratio", "reduction", "gearing", "gearratio")
DIMENSION_NAME_HINTS = (
    "diameter", "radius", "circumference", "wheel", "sprocket", "pulley",
    "drum", "spool", "pitchdiameter",
)


# --------------------------------------------------------------------------- #
# Exceptions
# --------------------------------------------------------------------------- #
class CredentialError(Exception):
    """Raised when API keys are not available. Carries NO secret material."""


class UsageError(Exception):
    """Bad CLI arguments / unparseable input."""


class QuotaExhausted(Exception):
    """HTTP 402 - the annual Onshape API quota is used up."""


class OnshapeHTTPError(Exception):
    def __init__(self, status, body=""):
        self.status = status
        self.body = body
        super().__init__(f"Onshape HTTP {status}")


# --------------------------------------------------------------------------- #
# Credentials & auth (no secret ever leaves this layer)
# --------------------------------------------------------------------------- #
def load_credentials(environ=None, local_file=None):
    """Return (access_key, secret_key). Env vars win; an optional gitignored
    JSON file ({"accessKey": "...", "secretKey": "..."}) is a fallback.
    Raises CredentialError (with no secret in the message) if unavailable."""
    environ = os.environ if environ is None else environ
    access = environ.get("ONSHAPE_ACCESS_KEY")
    secret = environ.get("ONSHAPE_SECRET_KEY")

    if not (access and secret) and local_file:
        p = Path(local_file)
        if p.is_file():
            try:
                data = json.loads(p.read_text(encoding="utf-8"))
                access = access or data.get("accessKey")
                secret = secret or data.get("secretKey")
            except (ValueError, OSError):
                pass

    if not (access and secret):
        raise CredentialError(
            "Onshape API keys not found. Set ONSHAPE_ACCESS_KEY and "
            "ONSHAPE_SECRET_KEY (create a read-only key at "
            "https://cad.onshape.com/user/developer/apiKeys). "
            "See onshape-integration.md > Setup."
        )
    return access, secret


def build_auth_header(access_key, secret_key):
    raw = f"{access_key}:{secret_key}".encode("utf-8")
    return "Basic " + base64.b64encode(raw).decode("ascii")


# --------------------------------------------------------------------------- #
# URL / id parsing
# --------------------------------------------------------------------------- #
_URL_RE = re.compile(
    r"documents/(?P<did>[0-9a-f]{24})"
    r"(?:/(?P<wvm>[wvm])/(?P<wvmid>[0-9a-f]{24}))?"
    r"(?:/e/(?P<eid>[0-9a-f]{24}))?",
    re.IGNORECASE,
)


def parse_onshape_url(url):
    """Parse a cad.onshape.com document URL into {did, wvm, wvmid, eid}.
    Preserves the w/v/m selector. Strips any query string first. eid may be
    None for a document-level link. Raises UsageError on garbage."""
    if not url:
        raise UsageError("empty Onshape URL")
    path = url.split("?", 1)[0].split("#", 1)[0]
    m = _URL_RE.search(path)
    if not m or not m.group("did"):
        raise UsageError(
            "could not parse an Onshape document id from the URL. Expected "
            "https://cad.onshape.com/documents/{did}/[w|v|m]/{wvmid}/e/{eid}"
        )
    return {
        "did": m.group("did"),
        "wvm": (m.group("wvm") or "w").lower(),
        "wvmid": m.group("wvmid"),
        "eid": m.group("eid"),
    }


def ids_from_args(args):
    if args.url:
        ids = parse_onshape_url(args.url)
    else:
        if not (args.did and args.wvmid):
            raise UsageError("provide --url, or --did and --wvmid (and --eid).")
        ids = {"did": args.did, "wvm": (args.wvm or "w"),
               "wvmid": args.wvmid, "eid": args.eid}
    if not ids.get("wvmid"):
        raise UsageError("missing workspace/version/microversion id (wvmid).")
    return ids


# --------------------------------------------------------------------------- #
# Small value parsers
# --------------------------------------------------------------------------- #
_QTY_RE = re.compile(r"^\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*(.*)$")


def parse_quantity_string(s):
    """'0.1016 meter' -> (0.1016, 'meter'); '9' -> (9.0, 'dimensionless');
    '4*inch' -> (4.0, 'inch'). Returns (None, None) if no leading number."""
    if s is None:
        return (None, None)
    m = _QTY_RE.match(str(s))
    if not m:
        return (None, None)
    value = float(m.group(1))
    unit = m.group(2).strip().lstrip("*").strip()
    return (value, unit or "dimensionless")


_NUM_EXPR_RE = re.compile(r"[0-9eE+\-*/.() ]+")


def safe_eval_ratio(expr):
    """Evaluate a PURELY NUMERIC arithmetic expression like '24/180' -> 0.1333.
    Onshape stores gear-relation ratios as expressions (often tooth counts).
    Returns None if the expression contains anything other than digits and
    arithmetic (e.g. it carries a unit like '1 in'), so the caller can fall
    back to unit parsing. Safe: the string is regex-validated to numerics only
    before eval, with builtins stripped."""
    if expr is None:
        return None
    s = str(expr).strip()
    if not s or not _NUM_EXPR_RE.fullmatch(s):
        return None
    try:
        return float(eval(s, {"__builtins__": {}}, {}))
    except Exception:
        return None


# --------------------------------------------------------------------------- #
# COTS candidate lookup (low-confidence only; markdown is the real authority)
# --------------------------------------------------------------------------- #
def load_cots_table(path=None):
    p = Path(path) if path else (Path(__file__).resolve().parent / "cots_ratios.json")
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def _clean_text(s):
    """Strip Unicode format/zero-width characters (category Cf, e.g. U+200E
    left-to-right mark seen in real Onshape BOM part names) so substring
    matching and ratio regexes aren't silently broken."""
    if s is None:
        return ""
    return "".join(ch for ch in str(s) if unicodedata.category(ch) != "Cf")


_LX_RE = re.compile(r"\bl([1-4])\b", re.IGNORECASE)
_PINION_RE = re.compile(r"\b(1[0-9]|9)\s*t\b", re.IGNORECASE)


def cots_candidate(name, table=None):
    """Given a BOM/part name, return a LOW-confidence COTS ratio candidate dict
    or None. Never authoritative - just a head start for Claude."""
    if not name:
        return None
    table = table if table is not None else load_cots_table()
    lower = " " + _clean_text(name).lower() + " "

    for key, fam in (table.get("swerve") or {}).items():
        if any(a in lower for a in fam.get("aliases", [])):
            drive = fam.get("driveRatios", {}) or {}
            code, value = None, None
            mlx = _LX_RE.search(lower)
            if mlx:
                code = "L" + mlx.group(1)
            else:
                mp = _PINION_RE.search(lower)
                if mp:
                    code = mp.group(0).replace(" ", "").upper()
            if code:
                for k, v in drive.items():
                    if k.lower() == code.lower():
                        value = v
                        break
            return {
                "family": key, "kind": "swerve", "code": code,
                "value": value, "steerRatio": fam.get("steerRatio"),
                "confidence": "low", "source": fam.get("source"),
                "note": "swerve drive ratio - prefer Tuner X / YAGSL; confirm config code",
            }

    for key, fam in (table.get("planetary") or {}).items():
        if any(a in lower for a in fam.get("aliases", [])):
            return {
                "family": key, "kind": "planetary", "code": None, "value": None,
                "cartridgeStages": fam.get("cartridgeStages"),
                "confidence": "low", "source": fam.get("source"),
                "note": fam.get("netRule", "net ratio = product of stacked cartridges"),
            }
    return None


# --------------------------------------------------------------------------- #
# Variables / BOM / config / mass-props / gear-relation parsers (pure)
# --------------------------------------------------------------------------- #
def _iter_variables(vars_json):
    if isinstance(vars_json, dict):
        if isinstance(vars_json.get("variables"), list):
            yield from vars_json["variables"]
        elif "name" in vars_json:
            yield vars_json
    elif isinstance(vars_json, list):
        for item in vars_json:
            if isinstance(item, dict) and isinstance(item.get("variables"), list):
                yield from item["variables"]
            elif isinstance(item, dict) and "name" in item:
                yield item


def parse_variables(vars_json):
    """Return (gear_ratios, dimensions) from an Onshape /variables response."""
    ratios, dims = [], []
    for v in _iter_variables(vars_json):
        name = (v.get("name") or "").strip()
        if not name:
            continue
        lname = name.lower().lstrip("#")
        expr = v.get("expression")
        value, unit = parse_quantity_string(v.get("value"))
        if value is None and expr is not None:
            value, unit = parse_quantity_string(expr)
        vtype = (v.get("type") or "").upper()

        is_dim = any(h in lname for h in DIMENSION_NAME_HINTS) or vtype == "LENGTH"
        is_ratio = any(h in lname for h in RATIO_NAME_HINTS)

        if is_ratio and value is not None:
            ratios.append({
                "value": value, "units": "dimensionless",
                "source": "variable", "sourceName": name,
                "confidence": "high", "raw": expr if expr is not None else v.get("value"),
                "context": "named CAD variable",
            })
        elif is_dim and value is not None:
            dims.append({
                "name": name, "value": value, "units": unit or "meter",
                "source": "variable", "confidence": "high",
                "raw": expr if expr is not None else v.get("value"),
            })
    return ratios, dims


def _header_id_to_name(bom_json):
    out = {}
    for h in bom_json.get("headers", []) or []:
        hid = h.get("id") or h.get("propertyId") or h.get("propertyName")
        hname = h.get("name") or h.get("propertyName") or hid
        if hid:
            out[hid] = hname
    return out


def parse_bom(bom_json, cots_table=None):
    """Return (rows, ratio_candidates) from an Onshape assembly BOM response."""
    cots_table = cots_table if cots_table is not None else load_cots_table()
    id2name = _header_id_to_name(bom_json)
    rows, candidates = [], []
    for r in bom_json.get("rows", []) or []:
        h2v = r.get("headerIdToValue") or r.get("headerIdToValueDisplay") or {}
        props = {}
        for hid, val in h2v.items():
            props[id2name.get(hid, hid)] = _flatten_cell(val)
        name = props.get("Name") or props.get("Part name") or ""
        part_no = props.get("Part number") or props.get("PartNumber") or ""
        desc = props.get("Description") or ""
        qty = props.get("Quantity") or props.get("QTY")
        haystack = _clean_text(" ".join(str(x) for x in (name, part_no, desc) if x))

        candidate = cots_candidate(haystack, cots_table)
        ratio = _regex_ratio(haystack)
        if ratio is not None:
            candidates.append({
                "value": ratio, "units": "dimensionless", "source": "bom-name",
                "sourceName": name or part_no, "confidence": "low",
                "raw": haystack, "context": "ratio parsed from BOM text",
            })

        rows.append({
            "name": name, "partNumber": part_no, "description": desc,
            "quantity": qty, "itemSource": r.get("itemSource"),
            "candidateRatio": candidate,
        })
    return rows, candidates


def _flatten_cell(val):
    if isinstance(val, dict):
        return val.get("value") or val.get("displayName") or val.get("name") or ""
    return val


_RATIO_RE = re.compile(r"(\d+(?:\.\d+)?)\s*:\s*1\b")


def _regex_ratio(text):
    m = _RATIO_RE.search(text or "")
    return float(m.group(1)) if m else None


def parse_config(cfg_json):
    """Surface configuration inputs and any ratio-looking candidates."""
    params, candidates = [], []
    current = {}
    for c in cfg_json.get("currentConfiguration", []) or []:
        cm = c.get("message", c)
        pid = cm.get("parameterId")
        if pid:
            current[pid] = cm.get("value") or cm.get("expression")
    for p in cfg_json.get("configurationParameters", []) or []:
        pm = p.get("message", p)
        pname = pm.get("parameterName") or pm.get("parameterId") or ""
        pid = pm.get("parameterId")
        params.append(pname)
        if any(h in pname.lower() for h in RATIO_NAME_HINTS):
            val, _ = parse_quantity_string(current.get(pid))
            if val is not None:
                candidates.append({
                    "value": val, "units": "dimensionless", "source": "config",
                    "sourceName": pname, "confidence": "medium",
                    "raw": current.get(pid), "context": "configuration input",
                })
    return {"configParameters": params}, candidates


def _walk_feature_lists(features_json):
    """Yield every feature dict from rootAssembly + all subAssemblies + a
    top-level 'features' list (the /features endpoint shape)."""
    if not isinstance(features_json, dict):
        return
    if isinstance(features_json.get("features"), list):
        yield from features_json["features"]
    root = features_json.get("rootAssembly")
    if isinstance(root, dict) and isinstance(root.get("features"), list):
        yield from root["features"]
    for sub in features_json.get("subAssemblies", []) or []:
        if isinstance(sub, dict) and isinstance(sub.get("features"), list):
            yield from sub["features"]


def _bt(obj):
    return str(obj.get("btType") or obj.get("typeName") or "")


def _is_mate_relation(feature, msg):
    return (
        "MateRelation" in _bt(feature)
        or "MateRelation" in _bt(msg)
        or (msg.get("featureType") == "mateRelation")
    )


def extract_gear_relations(features_json):
    """Extract gear / rack / screw relation ratios.

    Confirmed against live Onshape data (api-verified): a gear relation's ratio
    is parameterId 'relationRatio' (a dimensionless arithmetic expression such
    as '24/180' = tooth counts); rack/screw lead is 'relationLength' (distance
    per rev); 'relationType' selects GEAR/RACK_AND_PINION/SCREW/LINEAR;
    'reverseDirection' is the direction flag. Parameters are still located
    defensively by btType so a future id change degrades gracefully. For a GEAR
    relation we emit only the ratio (ignoring the inactive length default); for
    RACK/SCREW we emit the length. Returns (candidates, parameter_ids_seen)."""
    candidates, seen = [], []
    for feat in _walk_feature_lists(features_json):
        if not isinstance(feat, dict):
            continue
        # /features uses `message`; getAssemblyDefinition uses `featureData`.
        msg = feat.get("message") or feat.get("featureData") or feat
        if not _is_mate_relation(feat, msg):
            continue
        rel_name = msg.get("name") or feat.get("name") or "gear relation"
        rel_type, reverse = None, None
        quantities = {}  # parameterId (or synthetic) -> (expr, units)
        for p in msg.get("parameters", []) or []:
            pm = p.get("message", p)
            bt = _bt(p) or _bt(pm)
            pid = pm.get("parameterId") or p.get("parameterId")
            if pid:
                seen.append(pid)
            if "Enum" in bt:
                rel_type = pm.get("value") or pm.get("enumName") or pm.get("enumValue")
            elif "Bool" in bt:
                reverse = pm.get("value")
            elif "Quantity" in bt:
                expr = pm.get("expression")
                if expr is None and pm.get("value") is not None:
                    expr = str(pm.get("value"))
                quantities[pid or ("q%d" % len(quantities))] = (expr, pm.get("units"))

        rt = (rel_type or "").upper()
        is_linear = ("RACK" in rt) or ("SCREW" in rt)
        chosen = _choose_relation_quantities(quantities, is_linear)
        multi = len(chosen) > 1
        for pid, expr, units, kind in chosen:
            if kind == "ratio":
                value = safe_eval_ratio(expr)
                out_units = "dimensionless"
                if value is None:
                    value, out_units = parse_quantity_string(expr)
            else:
                value, out_units = parse_quantity_string(expr)
                out_units = out_units or units or "meter"
            verified_id = pid in ("relationRatio", "relationLength")
            conf = "high" if (kind == "ratio" and value is not None and verified_id) else "medium"
            candidates.append({
                "value": value, "units": out_units,
                "source": "relation", "sourceName": rel_name, "confidence": conf,
                "raw": expr, "parameterId": pid,
                "context": _relation_context(rel_type, reverse, multi, kind),
            })
    return candidates, sorted(set(seen))


def _choose_relation_quantities(quantities, is_linear):
    """Pick the meaningful quantity(ies): the length for rack/screw, else the
    ratio. Falls back to all quantities if ids are unrecognized. Returns a list
    of (pid, expr, units, kind)."""
    items = list(quantities.items())
    if is_linear:
        named = [(pid, e, u, "length") for pid, (e, u) in items
                 if pid == "relationLength" or "Length" in str(pid)]
        return named or [(pid, e, u, "length") for pid, (e, u) in items]
    named = [(pid, e, u, "ratio") for pid, (e, u) in items
             if pid == "relationRatio" or "Ratio" in str(pid)]
    return named or [(pid, e, u, "ratio") for pid, (e, u) in items]


def _relation_context(rel_type, reverse, multi, kind):
    bits = []
    if rel_type:
        bits.append(f"type={rel_type}")
    if reverse is not None:
        bits.append(f"reverse={reverse}")
    bits.append(f"value={kind}")
    if multi:
        bits.append("multiple quantities present")
    return "; ".join(bits)


def axis_moi_from_tensor(inertia, axis="z", mass=None, offset_m=0.0):
    """Pick the centroidal-axis MOI (Ixx/Iyy/Izz) from a 9- or 27-element
    Onshape inertia array and optionally apply the parallel-axis theorem.
    Returns a dict or None."""
    if not inertia or len(inertia) < 9:
        return None
    idx = {"x": 0, "y": 4, "z": 8}.get(axis, 8)
    i_centroidal = float(inertia[idx])
    applied = False
    i_axis = i_centroidal
    if offset_m and mass:
        i_axis = i_centroidal + float(mass) * float(offset_m) ** 2
        applied = True
    return {
        "momentOfInertia_kgm2": i_axis,
        "axis": axis,
        "centroidalMOI_kgm2": i_centroidal,
        "appliedParallelAxis": applied,
        "offset_m": float(offset_m or 0.0),
    }


def parse_massprops(mp_json, axis="z", offset_m=0.0):
    def first(key):
        v = mp_json.get(key)
        if isinstance(v, list) and v:
            return v
        return v

    mass_arr = mp_json.get("mass")
    mass = mass_arr[0] if isinstance(mass_arr, list) and mass_arr else mass_arr
    centroid = first("centroid")
    inertia = first("inertia")
    has_mass = mp_json.get("hasMass")
    missing = mp_json.get("massMissingCount", 0)

    moi = axis_moi_from_tensor(inertia, axis=axis, mass=mass, offset_m=offset_m)
    confidence = "high"
    warnings = []
    if has_mass is False or (missing or 0) > 0:
        confidence = "low"
        warnings.append(
            "mass/inertia unreliable: model has unassigned material(s); Onshape "
            "used a default density (massMissingCount=%s). Assign materials or "
            "verify against measured spin-up." % missing
        )
    out = {
        "mass_kg": mass,
        "centroid_m": centroid[:3] if isinstance(centroid, list) else centroid,
        "inertiaTensor_kgm2": inertia[:9] if isinstance(inertia, list) else inertia,
        "hasMass": has_mass,
        "massMissingCount": missing,
        "confidence": confidence,
    }
    if moi:
        out.update(moi)
    return out, warnings


# --------------------------------------------------------------------------- #
# HTTP client (cache + 402/429 + call counting)
# --------------------------------------------------------------------------- #
class OnshapeClient:
    def __init__(self, base_url, auth_header, cache_dir=None, no_cache=False,
                 timeout=30, max_retries=4, backoff_base=1.0, backoff_cap=8.0):
        self.base_url = base_url.rstrip("/")
        self.headers = {"Authorization": auth_header, "Accept": ACCEPT_HEADER}
        self.cache_dir = Path(cache_dir) if cache_dir else None
        self.no_cache = no_cache
        self.timeout = timeout
        self.max_retries = max_retries
        self.backoff_base = backoff_base
        self.backoff_cap = backoff_cap
        self.api_calls_used = 0

    # Seam for tests: monkeypatch this to avoid real network.
    def _open(self, req):
        return urllib.request.urlopen(req, timeout=self.timeout)

    def _cache_path(self, url):
        if not self.cache_dir:
            return None
        key = hashlib.sha1(url.encode("utf-8")).hexdigest()
        return self.cache_dir / (key + ".json")

    def _read_cache(self, path):
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None

    def _write_cache(self, path, data):
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(data), encoding="utf-8")
        except OSError:
            pass

    @staticmethod
    def _safe_body(err):
        try:
            return err.read().decode("utf-8", "replace")[:500]
        except Exception:
            return ""

    def get(self, endpoint_path, params=None):
        """GET a path under the base URL, returning parsed JSON. Uses the disk
        cache, increments api_calls_used only on a real network 2xx/3xx."""
        query = ("?" + urllib.parse.urlencode(params)) if params else ""
        url = self.base_url + endpoint_path + query

        cache_path = self._cache_path(url)
        if cache_path and not self.no_cache and cache_path.is_file():
            cached = self._read_cache(cache_path)
            if cached is not None:
                return cached

        attempts = 0
        while True:
            req = urllib.request.Request(url, headers=self.headers, method="GET")
            try:
                resp = self._open(req)
            except urllib.error.HTTPError as e:
                if e.code == 402:
                    raise QuotaExhausted()
                if e.code == 429 and attempts < self.max_retries:
                    time.sleep(self._retry_delay(e, attempts))
                    attempts += 1
                    continue
                raise OnshapeHTTPError(e.code, self._safe_body(e))
            except urllib.error.URLError as e:
                raise OnshapeHTTPError(0, str(getattr(e, "reason", e)))

            data = resp.read()
            self.api_calls_used += 1
            parsed = json.loads(data.decode("utf-8")) if data else {}
            if cache_path:
                self._write_cache(cache_path, parsed)
            return parsed

    def _retry_delay(self, err, attempts):
        ra = None
        try:
            ra = err.headers.get("Retry-After") if err.headers else None
        except Exception:
            ra = None
        if ra and str(ra).isdigit():
            return min(float(ra), self.backoff_cap)
        return min(self.backoff_base * (2 ** attempts), self.backoff_cap)


# --------------------------------------------------------------------------- #
# Endpoint helpers + per-mode fetchers
# --------------------------------------------------------------------------- #
def _seg(ids):
    return f"/d/{ids['did']}/{ids['wvm']}/{ids['wvmid']}/e/{ids['eid']}"


class Fetcher:
    def __init__(self, client, ids, args, cots_table):
        self.c = client
        self.ids = ids
        self.args = args
        self.cots = cots_table
        self.gear_ratios = []
        self.dimensions = []
        self.bom = []
        self.mass_properties = {}
        self.relation_param_ids = []
        self.warnings = []
        self.quota_exhausted = False

    def _config_param(self):
        return {"configuration": self.args.configuration} if self.args.configuration else None

    def run_variables(self):
        data = self.c.get(f"/variables{_seg(self.ids)}/variables", self._config_param())
        ratios, dims = parse_variables(data)
        self.gear_ratios.extend(ratios)
        self.dimensions.extend(dims)
        return any(r["confidence"] == "high" for r in ratios)

    def run_config(self):
        data = self.c.get(f"/elements{_seg(self.ids)}/configuration")
        _info, candidates = parse_config(data)
        self.gear_ratios.extend(candidates)

    def run_bom(self):
        params = {"indented": "false", "generateIfAbsent": "true"}
        if self.args.configuration:
            params["configuration"] = self.args.configuration
        data = self.c.get(f"/assemblies{_seg(self.ids)}/bom", params)
        rows, candidates = parse_bom(data, self.cots)
        self.bom = rows
        self.gear_ratios.extend(candidates)

    def run_relations(self):
        # The /features endpoint returns this assembly's top-level features
        # (each with a top-level btType + a `message` block) -- proven to carry
        # gear relations. NOTE: relations nested INSIDE a sub-assembly are not
        # returned here; to read those, point the helper at that sub-assembly's
        # own element URL.
        data = self.c.get(f"/assemblies{_seg(self.ids)}/features")
        candidates, seen = extract_gear_relations(data)
        self.gear_ratios.extend(candidates)
        self.relation_param_ids = seen

    def run_massprops(self):
        params = {"configuration": self.args.configuration} if self.args.configuration else None
        try:
            data = self.c.get(f"/assemblies{_seg(self.ids)}/massproperties", params)
        except OnshapeHTTPError:
            # Element may be a Part Studio, not an assembly - fall back.
            data = self.c.get(f"/partstudios{_seg(self.ids)}/massproperties", params)
        mp, warns = parse_massprops(
            data, axis=self.args.rotation_axis, offset_m=self.args.axis_offset_m
        )
        self.mass_properties = mp
        self.warnings.extend(warns)

    def run(self, mode):
        steps = {
            "variables": [self.run_variables],
            "config": [self.run_config],
            "bom": [self.run_bom],
            "relations": [self.run_relations],
            "massprops": [self.run_massprops],
        }
        try:
            if mode == "all":
                got_high = self._safe(self.run_variables) or False
                if not got_high:
                    self._safe(self.run_config)
                    self._safe(self.run_bom)
                    self._safe(self.run_relations)
                self._safe(self.run_massprops)
            else:
                for step in steps[mode]:
                    self._safe(step)
        except QuotaExhausted:
            self.quota_exhausted = True
            self.warnings.append(
                "annual Onshape API quota exhausted (HTTP 402) - stopped early. "
                "Use the screenshot fallback (onshape-integration.md)."
            )

    def _safe(self, step):
        """Run one step; let QuotaExhausted bubble, convert other HTTP errors
        into warnings so a partial result is still returned."""
        try:
            return step()
        except OnshapeHTTPError as e:
            self.warnings.append(f"{step.__name__}: Onshape HTTP {e.status} - skipped this source")
            return None


# --------------------------------------------------------------------------- #
# Output assembly
# --------------------------------------------------------------------------- #
def assemble_output(ids, mode, f):
    ratios = sorted(f.gear_ratios, key=lambda r: CONFIDENCE_ORDER.get(r.get("confidence"), 9))
    return {
        "input": {
            "url": None, "did": ids.get("did"), "wvm": ids.get("wvm"),
            "wvmid": ids.get("wvmid"), "eid": ids.get("eid"),
            "configuration": getattr(f.args, "configuration", None),
        },
        "mode": mode,
        "gearRatios": ratios,
        "dimensions": f.dimensions,
        "bom": f.bom,
        "massProperties": f.mass_properties,
        "relationParameterIdsSeen": f.relation_param_ids,
        "apiCallsUsed": f.c.api_calls_used,
        "quotaExhausted": f.quota_exhausted,
        "warnings": f.warnings,
    }


def resolve_latest_version(client, did):
    """Return the newest version id for a document, or None."""
    try:
        versions = client.get(f"/documents/d/{did}/versions")
    except (OnshapeHTTPError, QuotaExhausted):
        return None
    if not isinstance(versions, list) or not versions:
        return None
    dated = [v for v in versions if v.get("createdAt")]
    chosen = max(dated, key=lambda v: v["createdAt"]) if dated else versions[-1]
    return chosen.get("id")


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def build_parser():
    p = argparse.ArgumentParser(
        prog="onshape_fetch.py",
        description="Pull FRC gear ratios / dimensions / BOM / mass properties "
                    "from an Onshape document. Emits one JSON object to stdout.",
    )
    p.add_argument("--url", help="full cad.onshape.com document URL")
    p.add_argument("--did", help="document id (alternative to --url)")
    p.add_argument("--wvm", choices=["w", "v", "m"], default="w")
    p.add_argument("--wvmid", help="workspace/version/microversion id")
    p.add_argument("--eid", help="element (tab) id")
    p.add_argument("--mode", required=True,
                   choices=["variables", "config", "bom", "massprops", "relations", "all"])
    p.add_argument("--configuration", help="encoded non-default configuration string")
    p.add_argument("--prefer-version", action="store_true",
                   help="resolve a workspace URL to its latest version for cacheable reads")
    p.add_argument("--cache-dir", help="disk cache directory (default: ./.onshape-cache)")
    p.add_argument("--no-cache", action="store_true")
    p.add_argument("--rotation-axis", choices=["x", "y", "z"], default="z")
    p.add_argument("--axis-offset-m", type=float, default=0.0)
    p.add_argument("--base-url", default=DEFAULT_BASE_URL)
    p.add_argument("--timeout", type=float, default=30)
    p.add_argument("--credentials-file", help="optional gitignored JSON with accessKey/secretKey")
    p.add_argument("--json-only", action="store_true", help="suppress stderr notes")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)

    def note(msg):
        if not args.json_only:
            print(msg, file=sys.stderr)

    try:
        ids = ids_from_args(args)
    except UsageError as e:
        print(json.dumps({"error": str(e), "warnings": [str(e)]}))
        return 2

    try:
        access, secret = load_credentials(local_file=args.credentials_file)
    except CredentialError as e:
        print(json.dumps({"error": str(e), "warnings": [str(e)]}))
        return 2

    cache_dir = args.cache_dir or str(Path.cwd() / ".onshape-cache")
    client = OnshapeClient(
        base_url=args.base_url,
        auth_header=build_auth_header(access, secret),
        cache_dir=cache_dir, no_cache=args.no_cache, timeout=args.timeout,
    )
    del secret  # do not keep the secret around longer than needed

    if args.prefer_version and ids["wvm"] == "w":
        vid = resolve_latest_version(client, ids["did"])
        if vid:
            ids["wvm"], ids["wvmid"] = "v", vid
            note(f"using latest version {vid} for cacheable reads")

    f = Fetcher(client, ids, args, load_cots_table())
    f.run(args.mode)

    output = assemble_output(ids, args.mode, f)
    if args.url:
        output["input"]["url"] = args.url
    print(json.dumps(output, indent=2))

    if f.quota_exhausted:
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
