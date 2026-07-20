# Nonorthogonal Kinematics Derivation

source=configuration

For wheel `i`, drive direction `u_i=[cos(theta_i), sin(theta_i)]` and wheel-center position `r_i=[x_i,y_i]` give:

`v_i = cos(theta_i) vx + sin(theta_i) vy + (-y_i cos(theta_i) + x_i sin(theta_i)) wz`

Wheel angular velocity is `v_i / radius_i`; motor angular velocity additionally applies `gear_ratio_i * motor_sign_i`. The production motor matrix in canonical order is currently:

```text
FL  -21.426253656   25.534814771   2.016656569
FR   21.426253656   25.534814771   2.016656569
RL  -25.534814771  -21.426253656   2.016656569
RR   25.534814771  -21.426253656   2.016656569
```

Its singular values are `47.141772318`, `47.140452079`, and `4.017852638`. Rank is `3`; condition number is `11.733076488`. Forward kinematics uses `(A^T A)^-1 A^T`. Python and C precompute the matrix and pseudoinverse at initialization; neither performs trigonometry or pseudoinverse construction in the 100 Hz control tick.

Wheel saturation uses one common scale factor so chassis direction is preserved. The current geometry is mathematically nonsingular but remains mechanically unconfirmed.

## Python/C golden validation

The current-tree native C bridge links the production
`Core/Src/chassis_kinematics.c` and initializes it from the generated firmware
configuration. All `1011` deterministic Python vectors are checked in both
directions. The measured maximum Python/C motor-target difference is
`2.12170974496e-05 rad/s`; the maximum C inverse/forward round-trip component
error is `5.1908527432e-07`. The test is
`tests/test_chassis_kinematics_python_c_golden.py`.
