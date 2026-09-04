# Agent entrypoint

Use [`AGENTS.md`](./AGENTS.md) as the authoritative AI-coding and field-troubleshooting guide.

Fast checks:

```bash
bash scripts/field_diagnostics.sh
bash scripts/field_build_smoke.sh
```

Hardware constants:

```text
MID360 IP: 192.168.1.117
Bunker CAN: can0 @ 50000 bit/s
```

Do not let RTK correct `map->odom` in V1, do not bypass Bunker remote/manual priority, and do not remove the measured-stop gate before C1 capture.
