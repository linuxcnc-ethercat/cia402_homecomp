# CiA 402 homecomp test build

Test package for `cia402_homecomp`, built by CI from the `feat/sim-homing`
branch. It supports LinuxCNC 2.9 and 2.10 typed HAL APIs. Simulation CI is
green, but EtherCAT XML/SDO setup, drive-specific homing methods, and physical
machine safety have not been validated.

## 1. Select and install the package

Check the installed LinuxCNC version:

```sh
linuxcnc -v
```

Download the CI artifact matching that exact version:

- `cia402-homecomp-v2.9.10-amd64` for LinuxCNC 2.9.10.
- `cia402-homecomp-v2.10.0-pre2-amd64` for LinuxCNC 2.10.0-pre2.

Artifacts are attached to the successful `feat/sim-homing` run on the
[CI workflow page](https://github.com/linuxcnc-ethercat/cia402_homecomp/actions/workflows/ci.yml).
A GitHub login is required to download Actions artifacts. Extract the downloaded
ZIP, then install the contained Debian package:

```sh
sudo apt install ./cia402-homecomp_<version>_amd64.deb
```

The package installs only `/usr/lib/linuxcnc/modules/cia402_homecomp.so`.
It does not replace LinuxCNC, `cia402`, or the EtherCAT master.

Use the maintained `cia402` component from linuxcnc-ethercat v1.48.0 or later,
installed from the [linuxcnc-ethercat apt repository](https://github.com/linuxcnc-ethercat/apt).
Use the normal `main` flavor for LinuxCNC 2.9 and the HAL getter/setter flavor
required by the installed LinuxCNC 2.10 build. Do not load the prototype
`cia402.comp` alongside it.

Back up the machine configuration before editing it.

## 2. INI configuration

Select the module and retain normal final-position and following-error values:

```ini
[EMCMOT]
EMCMOT = motmod
HOMEMOD = cia402_homecomp

[JOINT_0]
HOME = 0.0
HOME_OFFSET = 0.0
HOME_FINAL_VEL = 10.0
HOME_SEARCH_VEL = 0.0
HOME_LATCH_VEL = 0.0
HOME_SEQUENCE = 0
FERROR = 1.0
MIN_FERROR = 0.5
```

Configure the CiA 402 homing method, speeds, and acceleration in the drive.
`HOME_OFFSET` assigns the joint coordinate at the discovered reference;
`HOME` is the coordinate targeted by LinuxCNC's normal final move.

## 3. HAL wiring

Remove any direct `joint.N.amp-enable-out -> cia402.N.enable` and
`cia402.N.pos-fb -> joint.N.motor-pos-fb` connections for joints using custom
homing. Route enable and feedback through homecomp instead. Example for joint
0:

```hal
net j0-command joint.0.motor-pos-cmd => cia402.0.pos-cmd joint.0.cia-pos-cmd
net j0-velocity-command joint.0.vel-cmd => cia402.0.velocity-cmd joint.0.cia-velocity-cmd
net j0-enable-command joint.0.amp-enable-out => joint.0.cia-enable-in
net j0-enable joint.0.cia-enable-out => cia402.0.enable
net j0-actual-position cia402.0.pos-fb => joint.0.cia-pos-fb-in
net j0-actual-velocity cia402.0.velocity-fb => joint.0.cia-velocity-fb-in
net j0-feedback joint.0.cia-pos-fb-out => joint.0.motor-pos-fb
net j0-home joint.0.cia-home => cia402.0.home
net j0-homed cia402.0.stat-homed => joint.0.cia-stat-homed
net j0-homing cia402.0.stat-homing => joint.0.cia-stat-homing
net j0-homing-error cia402.0.stat-homing-error => joint.0.cia-stat-homing-error
net j0-drive-fault cia402.0.stat-fault => joint.0.cia-stat-fault
net j0-op-enabled cia402.0.stat-op-enabled => joint.0.cia-stat-op-enabled
net j0-opmode your-drive.opmode-display => cia402.0.opmode-display joint.0.cia-opmode-display

setp joint.0.request-custom-homing 1
setp joint.0.cia-timeout-ms 30000
```

Replace `your-drive.opmode-display` with the actual EtherCAT PDO pin. Repeat
with the appropriate joint and `cia402` instance numbers for other joints.
Keep `request-custom-homing` separate from `cia402.N.home`; homecomp owns the
shared home request while custom homing is selected.

For a first test, enable custom homing on one joint only. Leave other joints on
standard homing by setting their `request-custom-homing` parameter to zero.

## 4. Test and report

**Use normal machine safety precautions. Keep E-stop available, clear travel,
and start with conservative drive speeds. The software timeout is not a
hardware safety function.**

Start LinuxCNC from a terminal so diagnostics are captured:

```sh
linuxcnc /path/to/machine.ini 2>&1 | tee homecomp-test.log
```

Home the selected joint and check:

1. The drive enters CiA 402 homing mode and moves without a LinuxCNC following
   error.
2. The joint becomes homed and finishes at the configured `HOME` coordinate.
3. Normal jogging works after homing.
4. Unhome and home again; the second attempt must perform a fresh drive home.

Report the drive make/model, homing method, LinuxCNC version, package
`BUILD-INFO`, changed INI/HAL files, and `homecomp-test.log`. Include any drive
fault code, especially Er87.1, and the `cia-state`/`home-state` values if homing
stops. Do not intentionally inject faults on an unsafe machine.

## Rollback

Stop LinuxCNC, restore the backed-up INI/HAL files, then remove the test module:

```sh
sudo dpkg -r cia402-homecomp
```
