# Mainline development policy

`agt_navigation_v3` is still in the design/field-validation stage. The repository intentionally uses a lightweight branch policy:

- `main` is the normal integration and development branch.
- Small fixes, parameter changes, adapters and incremental features land directly on `main`.
- Short-lived branches are reserved for destructive refactors, dependency replacements, or experiments that may temporarily break the field demo.
- Do not maintain parallel long-lived architecture branches during this stage.
- Hardware validation status must remain explicit in README/docs; merging to `main` means "current design baseline", not "hardware accepted".
- Once the robot passes repeatable field acceptance, introduce tags/releases and freeze upstream dependency SHAs for reproducibility.

The practical goal is to keep one obvious answer to: what is the current design, how is it started, what is unverified, and where should the next fix land?
