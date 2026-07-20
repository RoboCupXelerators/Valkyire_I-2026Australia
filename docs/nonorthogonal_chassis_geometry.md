# Nonorthogonal Chassis Geometry

source=configuration

## Coordinate system

- `+X`: robot forward
- `+Y`: robot left
- `+wz`: counterclockwise when viewed from above
- Canonical wheel order: `FL, FR, RL, RR`

## Current configuration

The wheel-center candidates are FL `+40 deg`, FR `-40 deg`, RL `+130 deg`, RR `-130 deg`. They encode the front `80 deg` and rear `100 deg` included angles. Wheel radius is temporarily `0.030 m`; wheel-center radius is approximately `0.0605 m`.

The current axis angles are radial and drive angles are axis `+90 deg`. This is a software-development hypothesis, not a mechanical measurement. All four ESC IDs have electronic motion evidence, but physical positive rolling directions and all four `motor_sign` values remain `TEMPORARY_UNCONFIRMED`; combined nonzero hardware output is hard-gated until that visual direction confirmation is completed.

The canonical config exposes calibration arrays in fixed `FL, FR, RL, RR` order: `wheel_radius_scale`, `motor_speed_scale`, `drive_angle_offset_deg`, `x_offset_m`, `y_offset_m`, and `motor_sign`. Their status is `UNMEASURED`; scales remain `1`, offsets remain `0`, and signs remain provisional. Both the Python loader and generated firmware header apply the same arrays, and invalid lengths, nonfinite values, nonpositive scales, or non-binary signs are rejected.

Motor mapping is verified independently of geometry:

| Wheel | ESC ID | Master ID |
|---|---:|---:|
| FL | 0x01 | 0x11 |
| FR | 0x02 | 0x12 |
| RL | 0x03 | 0x13 |
| RR | 0x04 | 0x14 |

The authoritative source is `config/chassis_geometry.json`. Firmware constants are generated from it and must not be edited as a second configuration source.
