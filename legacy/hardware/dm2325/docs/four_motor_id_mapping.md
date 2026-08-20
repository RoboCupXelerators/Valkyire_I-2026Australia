# Four-Motor ID Mapping

source=hardware

The canonical wheel order is fixed as `FL, FR, RL, RR`. CAN1 is the only
chassis bus; CAN2 is not used.

| Wheel | ESC ID | Master ID | Feedback CAN ID | Power-cycle verified |
| --- | ---: | ---: | ---: | --- |
| FL | `0x01` | `0x11` | `0x11` | yes |
| FR | `0x02` | `0x12` | `0x12` | yes |
| RL | `0x03` | `0x13` | `0x13` | yes |
| RR | `0x04` | `0x14` | `0x14` | yes |

Each motor was handled with only that motor powered. The original values and
raw replies were recorded, the approved values were saved, and both registers
were read again after a user-performed power cycle. The immutable raw records
are:

- `docs/dm2325_id_migration_FL.json`
- `docs/dm2325_id_migration_FR.json`
- `docs/dm2325_id_migration_RL.json`
- `docs/dm2325_id_migration_RR.json`

The later four-motor Disabled soak independently confirmed unique addressing,
independent feedback, `online_mask=0x0F`, no ID mismatch, and no CAN error. No
further ESC ID, Master ID, mode, baud-rate, or zero-point change is authorized.

Physical positive rolling direction is a separate property and remains
unconfirmed. It must not be inferred from these electronic identifiers.
