# Real 5 / 7 / 10 m and 30 m test observations

This CLI evaluates official field requirements and fails CLOSED when any
measurement, provenance or repetitions are missing.

- 5 m: average **surveyed planar terminal error <= 0.12 m**, five runs.
- 7 m: average **surveyed planar terminal error <= 0.15 m**, five runs.
- 10 m: average **surveyed planar terminal error <= 0.20 m**, five runs.
- 30 m speed gate: average speed **>= 1.0 m/s in each of two runs**.

This evaluates these navigation and speed metrics ONLY. A PASS does **not**
mean that 30° climbing, three-hour endurance, person stopping, mapping
accuracy or other acceptance criteria passed.

CSV header \`navigation.csv\`:

\`\`\`text
distance_m,run_id,target_x_m,target_y_m,arrival_x_m,arrival_y_m,measurement_source
\`\`\`

CSV header \`speed.csv\`:

\`\`\`text
run_id,gate_distance_m,elapsed_s,measurement_source
\`\`\`

Measurement provenance may be \`total_station\`, \`manual_tape\`, or
\`independent_survey\`. Do not copy FAST-LIO2/Nav2's own published position
as external measurement. Use one consistent independent local coordinate
system for both target and arrival survey positions. Only run the evaluation
after actual field observations exist:

\`\`\`bash
python3 field_ws/acceptance.py \
 --navigation measurements/navigation.csv \
 --speed measurements/speed.csv \
 --output experiments/acceptance_report.json
\`\`\`

Exit 0 means both tested criteria passed; 1 means FAIL; 2 INCOMPLETE; 3
invalid data. Missing CSV means zero recorded observations, never PASS.
