# System Identification

Fit a first-order motor model with delay to saved open-loop CSV logs. This is an
offline tool: it does not connect to or command the motor.

## Run

Use Python 3.13 or newer and `uv`. From the repository root:

```powershell
cd system_identification
uv sync --locked
uv run --locked run.py
```

Use the arrow keys to select a method, Enter to start, or Ctrl+C to cancel the
selection:

- `differential_evolution`: bounded search minimizing normalized RMSE.
- `least_square`: bounded least-squares fits using multiple initial delay
  candidates based on the CSV sample interval; the lowest-cost successful fit
  is selected.

## Input logs

By default, [run.py](run.py) reads CSV files directly inside
[assets/01_System_Identification/open-loop-responses](../assets/01_System_Identification/open-loop-responses).
Change `asset_dir` in `run.py` to use another input folder. The default path is
resolved relative to the script, independently of the launch directory.

Use one open-loop step response per file. Each CSV needs at least 10 rows and
these columns:

| Column | Unit |
| --- | --- |
| `Timestamp(ms)` | Milliseconds |
| `Commanded_PWM` | Signed PWM ticks |
| `Motor_Speed(RPM)` | Signed revolutions per minute |

Values must be numeric and finite. Timestamps must increase and have uniform
spacing within the input validator's numerical tolerance. The fitter derives
its sample interval from each CSV, rather than using the configured `DT_S`.
Speed is converted to pulses per second using the configured
`ROTATION_PER_PULSE`. Invalid files and unsuccessful fits are reported and
skipped while the remaining files are processed.

## Shared configuration

Edit [config/motor_config.toml](../config/motor_config.toml) for motor settings.
[Config/Motor0.py](Config/Motor0.py) reads the stored values and exposes the
existing Python names; it does not calculate derived values.

The TOML keys match Rust constant names in `SCREAMING_SNAKE_CASE`, including
`MOTOR_ID`, `D_S`, and `D_STEPS`. The Python loader also preserves the older
`motor_id`, `DELAY_TIME_S`, and `DELAY_STEPS` aliases. PID tables are named
`DEFAULT_PID_POS_CONFIG` and `DEFAULT_PID_SPEED_CONFIG`; their fields retain
Rust's `kp`, `ki`, `kd`, and `i_limit` names.

The formulas in the TOML are comments only. Update related values manually,
including pulse/rotation conversions, maximum speed in RPM, PWM ticks, sample
interval, and delay steps. Restart this script after an edit. The Rust desktop
application reads the same values during compilation, so rebuild it after
configuration changes. Python does not require a Rust build to read the TOML.

These settings do not modify firmware defaults or configuration stored on the
device. The fitting process estimates new `K`, `tau`, and `L` values; it does
not overwrite the configured linear-model parameters.

## Results and selecting a model

Successful fits are saved under
`system_identification/LOG/System_Identification<timestamp>/`:

- `system_identification_<method>_<timestamp>.csv`, with `PWM`, `K`, `tau`, and
  `L` columns. `K` is pulses/s per PWM tick; `tau` and `L` are seconds.
- `Motor_Linearity_<timestamp>.jpg`.
- `System_Identification_<timestamp>.jpg`.

If no fits succeed, no result files are saved. Missing linear regions are
reported and omitted from the region fits and speed-limit summaries; available
result points are still plotted.

Review the results before selecting them for simulation:

1. Update the chosen linear-model values in `config/motor_config.toml`, including
   the corresponding `D_STEPS` value.
2. To select a nonlinear dataset, copy the reviewed result CSV to
   [config/system_identification.csv](../config/system_identification.csv).
   The Rust loader requires at least two rows, strictly increasing unique PWM
   values, finite `PWM`, `K`, and `tau` values, and positive `tau`.
3. Rebuild `rust_script` to embed the selected settings and dataset.

Runs never replace either shared file automatically. The current Rust nonlinear
model interpolates gain from signed PWM; it uses the configured time constant
and delay rather than interpolating the CSV's `tau` or `L` columns.

See [System Identification](../docs/01-System-Identification.md) for the model,
historical experiment results, and simulation comparisons.
