#!/usr/bin/env python3
"""Offline unit tests for onshape_fetch.py. Standard library only (unittest);
no pip, no network. Run from the scripts dir:

    C:\\Python314\\python.exe -m unittest discover -s . -p "test_*.py"
or  C:\\Python314\\python.exe -m unittest test_onshape_fetch
"""
import base64
import email.message
import json
import os
import sys
import tempfile
import unittest
import urllib.error
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import onshape_fetch as of  # noqa: E402

FX = Path(__file__).resolve().parent / "fixtures"


def fixture(name):
    return json.loads((FX / name).read_text(encoding="utf-8"))


def http_error(code, retry_after=None):
    hdrs = email.message.Message()
    if retry_after is not None:
        hdrs["Retry-After"] = str(retry_after)
    return urllib.error.HTTPError("https://x", code, "err", hdrs, None)


class FakeResp:
    def __init__(self, payload):
        self._data = json.dumps(payload).encode("utf-8")

    def read(self):
        return self._data


# --------------------------------------------------------------------------- #
class TestUrlParsing(unittest.TestCase):
    DID, WID, EID = "0" * 24, "1" * 24, "2" * 24

    def test_workspace(self):
        ids = of.parse_onshape_url(
            f"https://cad.onshape.com/documents/{self.DID}/w/{self.WID}/e/{self.EID}")
        self.assertEqual(ids, {"did": self.DID, "wvm": "w", "wvmid": self.WID, "eid": self.EID})

    def test_version_and_microversion(self):
        self.assertEqual(of.parse_onshape_url(
            f"https://cad.onshape.com/documents/{self.DID}/v/{self.WID}/e/{self.EID}")["wvm"], "v")
        self.assertEqual(of.parse_onshape_url(
            f"https://cad.onshape.com/documents/{self.DID}/m/{self.WID}/e/{self.EID}")["wvm"], "m")

    def test_strips_query_and_keeps_selector(self):
        ids = of.parse_onshape_url(
            f"https://cad.onshape.com/documents/{self.DID}/v/{self.WID}/e/{self.EID}?configuration=foo")
        self.assertEqual(ids["wvm"], "v")
        self.assertEqual(ids["eid"], self.EID)

    def test_missing_element(self):
        ids = of.parse_onshape_url(f"https://cad.onshape.com/documents/{self.DID}/w/{self.WID}")
        self.assertIsNone(ids["eid"])

    def test_garbage_raises(self):
        with self.assertRaises(of.UsageError):
            of.parse_onshape_url("https://cad.onshape.com/documents/not-a-real-id")


class TestQuantityParsing(unittest.TestCase):
    def test_cases(self):
        self.assertEqual(of.parse_quantity_string("0.1016 meter"), (0.1016, "meter"))
        self.assertEqual(of.parse_quantity_string("9"), (9.0, "dimensionless"))
        self.assertEqual(of.parse_quantity_string("4*inch"), (4.0, "inch"))
        self.assertEqual(of.parse_quantity_string(None), (None, None))
        self.assertEqual(of.parse_quantity_string("abc"), (None, None))


class TestExprEval(unittest.TestCase):
    def test_safe_eval_ratio(self):
        self.assertAlmostEqual(of.safe_eval_ratio("24/180"), 24 / 180)
        self.assertEqual(of.safe_eval_ratio("9"), 9.0)
        self.assertAlmostEqual(of.safe_eval_ratio("(50/14)*(48/16)"), (50 / 14) * (48 / 16))
        self.assertIsNone(of.safe_eval_ratio("4*inch"))   # has a unit -> not pure numeric
        self.assertIsNone(of.safe_eval_ratio("1 in"))
        self.assertIsNone(of.safe_eval_ratio(None))
        self.assertIsNone(of.safe_eval_ratio("1/0"))      # ZeroDivisionError swallowed


class TestVariables(unittest.TestCase):
    def test_ratios_and_dimensions(self):
        ratios, dims = of.parse_variables(fixture("variables.json"))
        by_name = {r["sourceName"]: r for r in ratios}
        self.assertIn("gearRatio", by_name)
        self.assertEqual(by_name["gearRatio"]["value"], 9.0)
        self.assertEqual(by_name["gearRatio"]["confidence"], "high")
        self.assertEqual(by_name["shooterReduction"]["value"], 1.5)
        dims_by_name = {d["name"]: d for d in dims}
        self.assertIn("wheelDiameter", dims_by_name)
        self.assertAlmostEqual(dims_by_name["wheelDiameter"]["value"], 0.1016)
        self.assertEqual(dims_by_name["wheelDiameter"]["units"], "meter")


class TestCotsLookup(unittest.TestCase):
    def setUp(self):
        self.table = of.load_cots_table()

    def test_table_loaded(self):
        self.assertIn("swerve", self.table)
        self.assertIn("SDS_MK4", self.table["swerve"])

    def test_mk4_levels(self):
        for code, val in [("L1", 8.14), ("L2", 6.75), ("L3", 6.12), ("L4", 5.14)]:
            c = of.cots_candidate(f"SDS MK4 {code} module", self.table)
            self.assertEqual(c["family"], "SDS_MK4")
            self.assertEqual(c["value"], val)

    def test_mk4i_steer_and_drive(self):
        c = of.cots_candidate("MK4i Module L2", self.table)
        self.assertEqual(c["family"], "SDS_MK4i")
        self.assertEqual(c["value"], 6.75)
        self.assertAlmostEqual(c["steerRatio"], 21.4286)

    def test_maxswerve_pinions(self):
        for code, val in [("12T", 5.50), ("13T", 5.08), ("14T", 4.71)]:
            c = of.cots_candidate(f"REV MAXSwerve {code}", self.table)
            self.assertEqual(c["family"], "REV_MAXSwerve")
            self.assertEqual(c["value"], val)

    def test_planetary(self):
        c = of.cots_candidate("MAXPlanetary Gearbox", self.table)
        self.assertEqual(c["kind"], "planetary")
        self.assertIn(9, c["cartridgeStages"])

    def test_no_match(self):
        self.assertIsNone(of.cots_candidate("Aluminum bracket", self.table))

    def test_zero_width_unicode_does_not_break_match(self):
        # A real Onshape BOM contained a U+200E mark; matching must survive it.
        c = of.cots_candidate("MK4i‎ Module L2", self.table)
        self.assertIsNotNone(c)
        self.assertEqual(c["family"], "SDS_MK4i")
        self.assertEqual(c["value"], 6.75)


class TestBom(unittest.TestCase):
    def test_rows_and_candidates(self):
        rows, candidates = of.parse_bom(fixture("bom.json"))
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0]["candidateRatio"]["value"], 6.75)
        self.assertEqual(rows[0]["candidateRatio"]["family"], "SDS_MK4i")
        self.assertEqual(rows[2]["candidateRatio"]["kind"], "planetary")
        # The "9:1 reduction" text becomes a low-confidence bom-name candidate.
        bom_name = [c for c in candidates if c["source"] == "bom-name"]
        self.assertTrue(any(c["value"] == 9.0 for c in bom_name))


class TestGearRelations(unittest.TestCase):
    def test_defensive_extraction(self):
        candidates, seen = of.extract_gear_relations(fixture("features_gear_relation.json"))
        # GEAR -> one ratio candidate; RACK -> one length candidate. The GEAR
        # relation's inactive length default is NOT emitted.
        self.assertEqual(len(candidates), 2)
        gear = [c for c in candidates if c["sourceName"] == "Gear 1"][0]
        self.assertAlmostEqual(gear["value"], 24 / 180)        # arithmetic expression evaluated
        self.assertEqual(gear["units"], "dimensionless")
        self.assertEqual(gear["confidence"], "high")           # relationRatio id confirmed
        self.assertEqual(gear["parameterId"], "relationRatio")
        rack = [c for c in candidates if c["sourceName"] == "Rack relation"][0]
        self.assertAlmostEqual(rack["value"], 0.05)
        self.assertEqual(rack["units"], "meter")
        self.assertEqual(rack["parameterId"], "relationLength")
        # Sub-assembly relation was walked; all param ids surfaced.
        self.assertEqual(
            seen, ["matesQuery", "relationLength", "relationRatio", "relationType", "reverseDirection"])

    def test_plain_mate_ignored(self):
        only_mate = {"rootAssembly": {"features": [
            {"typeName": "BTMMate", "message": {"featureType": "mate", "parameters": []}}]}}
        candidates, seen = of.extract_gear_relations(only_mate)
        self.assertEqual(candidates, [])
        self.assertEqual(seen, [])

    def test_screw_relation_uses_length(self):
        feats = {"rootAssembly": {"features": [
            {"typeName": "BTMMateRelation", "message": {"featureType": "mateRelation", "name": "Screw 1", "parameters": [
                {"typeName": "BTMParameterEnum", "message": {"parameterId": "relationType", "value": "SCREW"}},
                {"typeName": "BTMParameterQuantity", "message": {"parameterId": "relationLength", "expression": "0.002*meter"}},
            ]}}]}}
        cands, _ = of.extract_gear_relations(feats)
        self.assertEqual(len(cands), 1)
        self.assertAlmostEqual(cands[0]["value"], 0.002)
        self.assertEqual(cands[0]["units"], "meter")
        self.assertEqual(cands[0]["parameterId"], "relationLength")

    def test_missing_parameter_id_falls_back(self):
        feats = {"rootAssembly": {"features": [
            {"typeName": "BTMMateRelation", "message": {"featureType": "mateRelation", "name": "Gear X", "parameters": [
                {"typeName": "BTMParameterEnum", "message": {"value": "GEAR"}},
                {"typeName": "BTMParameterQuantity", "message": {"expression": "2"}},
            ]}}]}}
        cands, seen = of.extract_gear_relations(feats)
        self.assertEqual(len(cands), 1)
        self.assertEqual(cands[0]["value"], 2.0)
        # no parameterId -> not the verified id -> medium confidence (not high)
        self.assertEqual(cands[0]["confidence"], "medium")


class TestMassProps(unittest.TestCase):
    def test_axis_moi_and_parallel_axis(self):
        inertia = [0.02, 0, 0, 0, 0.03, 0, 0, 0, 0.04]
        base = of.axis_moi_from_tensor(inertia, axis="z", mass=2.5, offset_m=0.0)
        self.assertEqual(base["momentOfInertia_kgm2"], 0.04)
        self.assertFalse(base["appliedParallelAxis"])
        shifted = of.axis_moi_from_tensor(inertia, axis="z", mass=2.5, offset_m=0.1)
        self.assertAlmostEqual(shifted["momentOfInertia_kgm2"], 0.065)
        self.assertTrue(shifted["appliedParallelAxis"])
        self.assertEqual(of.axis_moi_from_tensor([1, 2], "z")["momentOfInertia_kgm2"]
                         if of.axis_moi_from_tensor([1, 2], "z") else None, None)

    def test_parse_massprops_high(self):
        mp, warns = of.parse_massprops(fixture("massproperties.json"), axis="z")
        self.assertEqual(mp["mass_kg"], 2.5)
        self.assertEqual(mp["momentOfInertia_kgm2"], 0.04)
        self.assertEqual(mp["confidence"], "high")
        self.assertEqual(warns, [])

    def test_parse_massprops_missing_material(self):
        bad = {"mass": [0.0], "centroid": [0] * 9, "inertia": [0] * 9,
               "hasMass": False, "massMissingCount": 3}
        mp, warns = of.parse_massprops(bad)
        self.assertEqual(mp["confidence"], "low")
        self.assertTrue(warns)
        self.assertIn("material", warns[0])


class TestCredentials(unittest.TestCase):
    def test_missing_raises(self):
        with self.assertRaises(of.CredentialError):
            of.load_credentials(environ={})

    def test_present_returns(self):
        access, secret = of.load_credentials(
            environ={"ONSHAPE_ACCESS_KEY": "AKEY", "ONSHAPE_SECRET_KEY": "SEKRET"})
        self.assertEqual((access, secret), ("AKEY", "SEKRET"))

    def test_auth_header_roundtrip(self):
        header = of.build_auth_header("AKEY", "SUPERSECRET")
        self.assertTrue(header.startswith("Basic "))
        decoded = base64.b64decode(header.split(" ", 1)[1]).decode()
        self.assertEqual(decoded, "AKEY:SUPERSECRET")

    def test_secret_not_in_client_headers_plaintext(self):
        client = of.OnshapeClient("https://x", of.build_auth_header("AKEY", "SUPERSECRET"))
        self.assertNotIn("SUPERSECRET", json.dumps(client.headers))

    def test_credential_error_message_has_no_secret(self):
        try:
            of.load_credentials(environ={"ONSHAPE_ACCESS_KEY": "AKEY"})
        except of.CredentialError as e:
            self.assertNotIn("AKEY", str(e))


class TestHttpClient(unittest.TestCase):
    def _client(self, cache_dir=None):
        return of.OnshapeClient("https://cad.onshape.com/api/v10",
                                of.build_auth_header("a", "b"),
                                cache_dir=cache_dir)

    def test_402_raises_quota(self):
        client = self._client()
        client._open = lambda req: (_ for _ in ()).throw(http_error(402))
        with self.assertRaises(of.QuotaExhausted):
            client.get("/anything")

    def test_429_backoff_then_success(self):
        client = self._client()
        seq = [http_error(429, retry_after=0), http_error(429, retry_after=0),
               FakeResp({"ok": True})]

        def fake_open(req):
            item = seq.pop(0)
            if isinstance(item, Exception):
                raise item
            return item
        client._open = fake_open
        slept = []
        orig_sleep = of.time.sleep
        of.time.sleep = lambda s: slept.append(s)
        try:
            result = client.get("/x")
        finally:
            of.time.sleep = orig_sleep
        self.assertEqual(result, {"ok": True})
        self.assertEqual(client.api_calls_used, 1)
        self.assertEqual(len(slept), 2)

    def test_cache_hit_does_not_increment(self):
        with tempfile.TemporaryDirectory() as d:
            client = self._client(cache_dir=d)
            calls = []
            client._open = lambda req: (calls.append(1), FakeResp({"v": 1}))[1]
            first = client.get("/cached/path")
            second = client.get("/cached/path")
            self.assertEqual(first, second)
            self.assertEqual(len(calls), 1)          # network hit once
            self.assertEqual(client.api_calls_used, 1)


class TestFetcherOutput(unittest.TestCase):
    def _args(self, mode="all"):
        url = "https://cad.onshape.com/documents/%s/w/%s/e/%s" % ("0" * 24, "1" * 24, "2" * 24)
        return of.build_parser().parse_args(["--mode", mode, "--url", url])

    def test_short_circuits_on_high_confidence_variable(self):
        class FakeClient:
            def __init__(self):
                self.api_calls_used = 0

            def get(self, path, params=None):
                self.api_calls_used += 1
                if "/variables" in path:
                    return fixture("variables.json")
                if "/massproperties" in path:
                    return fixture("massproperties.json")
                raise AssertionError("should not fetch %s after a high-confidence hit" % path)

        args = self._args("all")
        ids = of.ids_from_args(args)
        f = of.Fetcher(FakeClient(), ids, args, of.load_cots_table())
        f.run("all")
        out = of.assemble_output(ids, "all", f)
        for key in ("input", "mode", "gearRatios", "dimensions", "bom",
                    "massProperties", "relationParameterIdsSeen", "apiCallsUsed",
                    "quotaExhausted", "warnings"):
            self.assertIn(key, out)
        self.assertEqual(out["mode"], "all")
        self.assertEqual(out["apiCallsUsed"], 2)            # variables + massprops only
        self.assertFalse(out["quotaExhausted"])
        self.assertEqual(out["massProperties"]["momentOfInertia_kgm2"], 0.04)
        confidences = [r["confidence"] for r in out["gearRatios"]]
        self.assertEqual(confidences, sorted(confidences, key=lambda c: of.CONFIDENCE_ORDER[c]))

    def test_full_cascade_when_no_high_confidence(self):
        class FakeClient:
            def __init__(self):
                self.api_calls_used = 0

            def get(self, path, params=None):
                self.api_calls_used += 1
                if "/variables" in path:
                    return []                              # no variables -> no high hit
                if "/configuration" in path:
                    return {"configurationParameters": [], "currentConfiguration": []}
                if "/bom" in path:
                    return fixture("bom.json")
                if "/massproperties" in path:
                    return fixture("massproperties.json")
                # bare assembly-definition endpoint (relations)
                return fixture("features_gear_relation.json")

        args = self._args("all")
        ids = of.ids_from_args(args)
        f = of.Fetcher(FakeClient(), ids, args, of.load_cots_table())
        f.run("all")
        out = of.assemble_output(ids, "all", f)
        self.assertEqual(out["apiCallsUsed"], 5)           # vars+config+bom+features+massprops
        sources = {r["source"] for r in out["gearRatios"]}
        self.assertIn("relation", sources)
        self.assertIn("bom-name", sources)
        self.assertEqual(len(out["bom"]), 3)


class TestQuotaStops(unittest.TestCase):
    def test_quota_exhausted_sets_flag(self):
        class FakeClient:
            def __init__(self):
                self.api_calls_used = 0

            def get(self, path, params=None):
                raise of.QuotaExhausted()

        args = of.build_parser().parse_args(
            ["--mode", "variables", "--url",
             "https://cad.onshape.com/documents/%s/w/%s/e/%s" % ("0" * 24, "1" * 24, "2" * 24)])
        ids = of.ids_from_args(args)
        f = of.Fetcher(FakeClient(), ids, args, {})
        f.run("variables")
        out = of.assemble_output(ids, "variables", f)
        self.assertTrue(out["quotaExhausted"])
        self.assertTrue(any("quota" in w.lower() for w in out["warnings"]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
