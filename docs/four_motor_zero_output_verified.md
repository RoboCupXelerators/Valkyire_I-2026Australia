# Four-Motor Zero-Output Verification

source=hardware

Status: **PASS on v2; historical FAIL artifacts retained**.

The post-mailbox-fix one-shot run delivered 3000 strict-zero chassis commands over `30.000060714 s`. All four Enable transitions succeeded; parser errors, unexpected nonzero targets, CAN bus-off, and fault masks remained zero. STOPPING then entered runtime fault 7 because the first Disable set did not produce confirmation before the 150 ms deadline. The emergency path sent Disable again and all four motors became physically Disabled.

The failure is preserved in:

- `docs/four_motor_zero_output_verified_30s.csv`
- `docs/four_motor_zero_output_verified_30s.txt`
- `docs/four_motor_zero_output_verified_acceptance.json`

Firmware now performs one bounded Disable confirmation retry at 75 ms, covered by C fault-injection tests. Historical FAIL evidence remains unchanged.

After that targeted fix, the permitted v2 zero-output run completed `30.000061958 s` and 3000 strict-zero commands. All four Enable transitions succeeded; stale, parser, ID mismatch, CAN error, bus-off, and unexpected nonzero counts were zero. Maximum command skew was `130 us`, scheduler lateness maximum was zero, and all four motors finished Disabled. Evidence is in `docs/four_motor_zero_output_verified_v2_30s.csv`, `.txt`, and `_acceptance.json`.
