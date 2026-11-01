# CiA 402 drive-controlled homing for LinuxCNC

This repository provides a LinuxCNC `homemod` for drives that execute CiA 402
homing methods internally.  It is a fork of Rod Webster's prototype and keeps
the original GPL-2.0 license and history.

The production drive interface is the maintained `cia402` component from
[linuxcnc-ethercat](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat).
The older `cia402.comp` and `basecomp.comp` files in this repository are
historical references, not additional production implementations.

## Architecture and ownership

Drive-controlled homing moves the motor without a LinuxCNC trajectory command.
Ordinary following-error monitoring therefore sees a growing error unless the
feedback path is handled explicitly.

This module splits responsibilities as follows:

- `cia402` is the only owner of the drive controlword, requested operation
  mode, target position, and target velocity.
- `cia402_homecomp` owns the LinuxCNC homing sequence, the shared
  `cia402.N.home` request, and the enable/feedback pass-through pins.
- During drive-owned homing only, `cia-pos-fb-out` follows `cia-pos-cmd`.
  Actual drive feedback remains available on `cia-pos-fb-in`.
- Completion requires a fresh homed indication after the new attempt has been
  observed unhomed.  A stale `cia402.stat-homed` cannot complete a rehome.
  Statusword bits 10 and 12 are decoded by `cia402`; homing error takes
  precedence, including when target-reached is also set.
- The module waits for rotary unlock and drive operation-enabled before
  asserting the homing request.  Drive faults and the configured timeout also
  abort an unlock wait.  `LOCKING_INDEXER` retains its normal unlock/relock
  behavior.  An explicit standard-homing retry also clears abort inhibition.
- `HOME_ABSOLUTE_ENCODER` delegates to LinuxCNC's base homing path instead of
  starting drive-owned motion, preserving its no-rehome/no-final-move behavior.
- On success, the module reconciles motor offset, actual feedback, command,
  and the free planner, then delegates the normal final move to LinuxCNC.
  `HOME` and `HOME_OFFSET` therefore keep their standard meaning.
- On fault, homing error, timeout, or cancellation, it withdraws the home
  request, synchronizes to actual position, enters LinuxCNC's normal abort
  path, and revokes drive enable until an explicit homing retry.  Active
  same-sequence CiA peers are revoked immediately, including peers in base
  finalization.  Standard-peer failures trigger cleanup in the same servo
  invocation.  On LinuxCNC 2.10, a bounded-move return stops drive-owned
  motion while preserving the base coordinated return states.
- Joints with `request-custom-homing` cleared use LinuxCNC's standard homing
  state machine, so mixed configurations are supported.

Feedback suppression removes LinuxCNC's following-error protection during the
drive-owned interval.  Configure the drive's own travel, torque/current,
sensor, and homing-error protections, and set a finite `cia-timeout-ms`.
This software timeout is not a hardware safety function.

No LinuxCNC core patch is required.  The module does include the matching
LinuxCNC `src/emc/motion/homing.c`, so it must be rebuilt against the source
version that supplies the running `motmod`.

## Build

Build LinuxCNC as a run-in-place tree, then provide both source paths:

```sh
LINUXCNC_SRC=/path/to/linuxcnc \
LINUXCNC_ETHERCAT_SRC=/path/to/linuxcnc-ethercat \
scripts/build.sh
```

With sibling checkouts named `linuxcnc`, `linuxcnc-ethercat`, and
`cia402_homecomp`, the paths are detected automatically.  The script installs
`cia402_homecomp.so`, the real `cia402.so`, and the test-only virtual drive
into the selected LinuxCNC RIP `rtlib` directory.  No root privileges or
EtherCAT hardware are needed for the simulation build.

Successful CI runs publish amd64 test packages for LinuxCNC 2.9.10 and
2.10.0-pre2 as workflow artifacts.  See [TESTING.md](TESTING.md) for package
installation, machine wiring, first-test checks, and rollback instructions.
These packages contain only the homecomp module; install the matching
linuxcnc-ethercat `cia402` package separately.

The build detects the 2.9 versus 2.10 homing-parameter API.  CI tests
LinuxCNC `v2.9.10` and `v2.10.0-pre2`, with an uncached master canary.

## INI setup

Select the module and give each drive-homed joint normal final-position and
following-error settings:

```ini
[EMCMOT]
EMCMOT = motmod
HOMEMOD = cia402_homecomp
SERVO_PERIOD = 1000000

[JOINT_0]
HOME = 25.0
HOME_OFFSET = 5.0
HOME_FINAL_VEL = 10.0
HOME_SEARCH_VEL = 0.0
HOME_LATCH_VEL = 0.0
HOME_SEQUENCE = 0
FERROR = 0.25
MIN_FERROR = 0.10
```

The drive's own CiA 402 homing method, speeds, and acceleration are configured
in the drive, not through `HOME_SEARCH_VEL` or `HOME_LATCH_VEL`.

## HAL wiring

Connect the normal EtherCAT PDO pins to `cia402` as usual.  The additional
homing wiring for joint 0 is:

```hal
net j0-command joint.0.motor-pos-cmd => cia402.0.pos-cmd joint.0.cia-pos-cmd
net j0-velocity-command joint.0.vel-cmd => cia402.0.velocity-cmd joint.0.cia-velocity-cmd
net j0-enable-command joint.0.amp-enable-out => joint.0.cia-enable-in
net j0-enable joint.0.cia-enable-out => cia402.0.enable
net j0-actual-position cia402.0.pos-fb => joint.0.cia-pos-fb-in
net j0-actual-velocity cia402.0.velocity-fb => joint.0.cia-velocity-fb-in
net j0-feedback joint.0.cia-pos-fb-out => joint.0.motor-pos-fb
net j0-velocity-feedback joint.0.cia-velocity-fb-out
net j0-home joint.0.cia-home => cia402.0.home
net j0-homed cia402.0.stat-homed => joint.0.cia-stat-homed
net j0-homing cia402.0.stat-homing => joint.0.cia-stat-homing
net j0-homing-error cia402.0.stat-homing-error => joint.0.cia-stat-homing-error
net j0-drive-fault cia402.0.stat-fault => joint.0.cia-stat-fault
net j0-op-enabled cia402.0.stat-op-enabled => joint.0.cia-stat-op-enabled
net drive-opmode-display your-drive.opmode-display => cia402.0.opmode-display joint.0.cia-opmode-display

setp joint.0.request-custom-homing 1
setp joint.0.cia-timeout-ms 30000
```

Replace `your-drive.opmode-display` with the EtherCAT PDO source used by the
configuration.  The complete executable example, including PDO-side virtual
drive connections and one standard-homing joint, is
[`tests/sim/test.hal`](tests/sim/test.hal).

### Homecomp pins

Replace `N` with the joint number.

| Pin | Direction | Purpose |
| --- | --- | --- |
| `joint.N.request-custom-homing` | IN | Select drive-controlled homing for this joint |
| `joint.N.is-custom-homing` | OUT | Echo of the selection input |
| `joint.N.cia-enable-in` | IN | Normally `joint.N.amp-enable-out` |
| `joint.N.cia-enable-out` | OUT | Drive enable with abort inhibition |
| `joint.N.cia-pos-cmd` | IN | LinuxCNC motor position command |
| `joint.N.cia-velocity-cmd` | IN | LinuxCNC joint velocity command |
| `joint.N.cia-pos-fb-in` | IN | Actual scaled feedback from `cia402` |
| `joint.N.cia-velocity-fb-in` | IN | Actual scaled velocity from `cia402` |
| `joint.N.cia-pos-fb-out` | OUT | Position feedback presented to LinuxCNC/PID |
| `joint.N.cia-velocity-fb-out` | OUT | Velocity feedback for an optional PID loop |
| `joint.N.cia-home` | IO | Shared home request; `cia402` clears it on the homed edge |
| `joint.N.cia-stat-homed` | IN | Bits 10+12 decoded by `cia402` |
| `joint.N.cia-stat-homing` | IN | Diagnostic drive-homing status |
| `joint.N.cia-stat-homing-error` | IN | Homing-mode statusword bit 13 |
| `joint.N.cia-stat-fault` | IN | Drive fault statusword bit 3 |
| `joint.N.cia-stat-op-enabled` | IN | Drive operation-enabled status; required before homing starts |
| `joint.N.cia-opmode-display` | IN | Actual drive operation mode |
| `joint.N.cia-timeout-ms` | IN | Homing timeout; zero disables it, default 30000 |
| `joint.N.cia-state` | OUT | 0 idle, 1 mode wait, 2 running, 3 base finalization, 4 enable wait, 5 unlock wait |
| `joint.N.cia-peer-abort` | OUT | Latched until retry after sequence peer-abort cleanup |

## Headless simulation

Run:

```sh
LINUXCNC_SRC=/path/to/linuxcnc tests/sim/test.sh
```

The test starts real `motmod`, `trivkins`, `milltask`, two `cia402` instances,
and `cia402_homecomp` without a GUI.  Two C realtime HAL components simulate
the PDO-side drives, including power-state sequencing and strict homing-start
edges.  They advance from servo cycles, not wall-clock delays.  Dedicated
headless configurations cover standard-peer abort, rotary locking, and
absolute-encoder base homing.

Covered scenarios:

- Large drive-owned motion without a false LinuxCNC following error
- Mode-6 acknowledgement, bits 10+12 success, and CSP restoration
- Nonzero `HOME` and `HOME_OFFSET`, final move, fresh-edge rehoming, and backlash
- Strict operation-enabled retry after cancellation
- Homing error with target-reached asserted simultaneously
- Drive fault and explicit fault reset
- Stall timeout and user cancellation with enable revocation
- Concurrent mixed drive-controlled/standard homing and synchronized final moves
- Active two-drive peer abort during drive motion and base finalization
- Standard-homing retry after an aborted drive-homing attempt
- Standard-peer abort of an active CiA joint, including reversed joint order
  and shared-switch rejection before `H[].homing` is set on 2.9
- Coordinated `HOME_RETURN_START/WAIT` handling after a bounded 2.10 move fails
- Rotary unlock/relock, unlock timeout/fault handling, and absolute-encoder
  no-final/no-rehome base homing

The simulation validates homing logic and LinuxCNC integration.  It does not
validate EtherCAT XML, SDO configuration, PDO transport, drive-specific homing
methods, hardware limits, or physical safety.  Hardware validation remains a
separate required step.
