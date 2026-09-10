# ATI gravity calibration data

Calibration date: 2026-09-10 (Asia/Shanghai)

## Files

- `ft_gravity_calibration.yaml`: accepted calibration generated from the last
  10 consistent poses. This is the file consumers should load.
- `ft_gravity_calibration.mixed-18.yaml`: all 18 persisted samples from both
  acquisition batches. It is retained as raw evidence; its combined fit is
  invalid because the first 8 samples are inconsistent with the later batch.
- `ft_gravity_calibration.invalid-20deg.yaml`: earlier 12-pose, 20-degree
  calibration result retained for comparison. It is invalid and must not be
  used for compensation.

## Accepted result

- Downstream mass: `3.2170005989 kg`
- Center of mass in `ft_sensor`: `[-0.0739583, 0.0746015, 0.0594935] m`
- Force bias: `[5.7368001, 7.2738515, 32.9430006] N`
- Torque bias: `[-0.2363989, -0.1070706, -0.2677242] N m`
- Force RMS residual: `0.7986392 N`
- Torque RMS residual: `0.0916483 N m`
- Design condition number: `9.4482`

The ATI hardware bias must remain zero. Gravity compensation must use the
software parameters above. The mass is the load below the sensing element;
it is not the total robot payload including the Net F/T sensor body.

The accepted result is a fit result. Validate it on poses not used by the fit
before enabling collision detection or force control.
