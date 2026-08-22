# Agent Ingest Instructions

Use this when Codex or Claude is asked to process new files in `Autonomous-Data-Collection`.

## Required Reading Order

1. `C:\FRC\CLAUDE.md`
2. `C:\FRC\Helios-2026-Robot-Data\README.md`
3. `C:\FRC\Helios-2026-Robot-Data\Hardware-Data-Sheet.md`
4. `C:\FRC\Helios-2026-Robot-Data\Rewrite-Checklist.md`
5. This folder's `README.md`

## Ingest Steps

1. Inventory every new file in `inbox/`, grouped by session date and evidence source.
2. Create `findings/YYYY-MM-DD-ingest-summary.md`.
3. For each fact, record:
   - value,
   - units,
   - source file,
   - evidence tag,
   - confidence,
   - whether it changes code.
4. Compare new facts against `Hardware-Data-Sheet.md`.
5. Separate conflicts from confirmations.
6. Update the main data sheet only after preserving the raw source path in the note.
7. List code TODOs that should be changed in `C:\FRC\Helios-2026`.

## Conflict Rules

- Vendor tool readback beats photo inference.
- Robot measurement beats CAD.
- Generated Tuner X files beat old copied constants for swerve, but CANcoder offsets still require re-zeroing on the robot.
- A Driver Station log proves symptoms, not root cause, unless correlated with robot telemetry.
- A screenshot is acceptable evidence if the relevant value is visible and the file name/session identifies when it was taken.

## Output Template

```markdown
# YYYY-MM-DD Ingest Summary

## New Evidence

| File | Source Type | Summary |
|---|---|---|

## Confirmed Facts

| Fact | Value | Evidence Tag | Source |
|---|---|---|---|

## Conflicts

| Existing Value | New Value | Likely Resolution | Source |
|---|---|---|---|

## Code Impact

| Area | Required Change | Risk |
|---|---|---|

## Still Unknown

| Question | Best Next Data Source |
|---|---|
```
