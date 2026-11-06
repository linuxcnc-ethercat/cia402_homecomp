#!/usr/bin/env python3
"""Headless tests for rotary unlock during drive-controlled homing."""

import sys
import time

import hal
import linuxcnc


h = hal.component("cia402-flags-test-ui")
h.ready()
c = linuxcnc.command()
s = linuxcnc.stat()
e = linuxcnc.error_channel()
failures = []
messages = []
unlock_since = None
unlock_seen = False
premature_home = False
auto_unlock = True


def check(condition, message):
    if not condition:
        failures.append(message)
        print("FAIL: " + message, flush=True)


def value(name):
    return hal.get_value(name)


def setp(name, new_value):
    hal.set_p(name, str(new_value))


def poll():
    global unlock_since, unlock_seen, premature_home, auto_unlock
    s.poll()
    while True:
        item = e.poll()
        if not item:
            break
        messages.append(item[1])
        print("linuxcnc error %d: %s" % (item[0], item[1]), flush=True)
    if value("joint.0.unlock"):
        if unlock_since is None:
            unlock_since = time.time()
        unlock_seen = True
        if not value("joint.0.is-unlocked") and value("joint.0.cia-home"):
            premature_home = True
        if auto_unlock and time.time() - unlock_since >= 0.1:
            setp("joint.0.is-unlocked", 1)
    else:
        unlock_since = None
        if value("joint.0.is-unlocked"):
            setp("joint.0.is-unlocked", 0)


def wait_for(condition, timeout=8.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        poll()
        if condition():
            return True
        time.sleep(0.002)
    poll()
    return condition()


def wait_for_homing_end(timeout=10.0):
    deadline = time.time() + timeout
    quiet_since = None
    while time.time() < deadline:
        poll()
        if any(j["homing"] for j in s.joint[:2]):
            quiet_since = None
        else:
            if quiet_since is None:
                quiet_since = time.time()
            if time.time() - quiet_since >= 0.2:
                return True
        time.sleep(0.002)
    return False


def machine_on():
    time.sleep(0.2)
    c.state(linuxcnc.STATE_ESTOP_RESET)
    c.wait_complete()
    wait_for(lambda: s.task_state == linuxcnc.STATE_ESTOP_RESET, 2.0)
    c.state(linuxcnc.STATE_ON)
    c.wait_complete()
    c.mode(linuxcnc.MODE_MANUAL)
    c.wait_complete()
    if not wait_for(lambda: s.task_state == linuxcnc.STATE_ON, 5.0):
        check(False, "machine did not reach STATE_ON")
        sys.exit(1)


def begin_home(joint):
    c.teleop_enable(0)
    c.wait_complete()
    c.home(joint)
    c.wait_complete()
    return wait_for(lambda: any(j["homing"] for j in s.joint[:2]) or
                    value("joint.0.cia-opmode-display") == 6, 3.0)


def finish_home(joint):
    check(begin_home(joint), "homing did not start for joint %d" % joint)
    ended = wait_for_homing_end()
    time.sleep(0.1)
    poll()
    check(ended, "homing did not finish for joint %d" % joint)
    return ended


def finish_immediate_failure(joint):
    """A pre-existing drive fault may abort before Python sees homing."""
    c.teleop_enable(0)
    c.wait_complete()
    c.home(joint)
    c.wait_complete()
    ended = wait_for_homing_end()
    time.sleep(0.1)
    poll()
    check(ended, "immediate homing failure did not finish for joint %d" % joint)


def unhome(joint):
    c.teleop_enable(0)
    c.wait_complete()
    c.unhome(joint)
    c.wait_complete()
    check(wait_for(lambda: not s.homed[joint]),
          "joint %d did not unhome" % joint)


def check_failed_home(label):
    check(not s.homed[0], label + " falsely marked joint 0 homed")
    check(not value("joint.0.cia-home"), label + " left home request asserted")
    check(value("joint.0.cia-enable-out") == value("joint.0.cia-enable-in"),
          label + " did not restore normal joint enable control")


def check_locked_at_home(label):
    check(s.homed[0], "%s did not mark joint 0 homed" % label)
    check(abs(s.joint[0]["input"] - 25.0) < 0.02,
          "%s reached %.6f instead of HOME 25.0" %
          (label, s.joint[0]["input"]))
    check(abs(value("joint.0.cia-pos-fb-in") - 120.0) < 0.02,
          "%s reached motor position %.6f instead of 120.0" %
          (label, value("joint.0.cia-pos-fb-in")))
    check(unlock_seen, "%s never requested rotary unlock" % label)
    check(not premature_home, "%s asserted homing before unlock" % label)
    check(not value("joint.0.unlock") and not value("joint.0.is-unlocked"),
          "%s did not relock rotary after homing" % label)


machine_on()
setp("cia402-drive.0.scenario", 0)
setp("cia402-drive.0.homing-ticks", 40)

# Exercise delayed unlock, the normal final move, and final relock.
finish_home(0)
check_locked_at_home("locking drive home")
check(not any("following error" in m.lower() for m in messages),
      "locking homing caused a LinuxCNC following error")

# A missing unlock must not bypass timeout accounting or strand drive enable.
unhome(0)
auto_unlock = False
setp("joint.0.cia-timeout-ms", 100)
finish_home(0)
check_failed_home("unlock timeout")

# A drive fault must also abort while still waiting for rotary unlock.
auto_unlock = False
setp("joint.0.cia-timeout-ms", 1000)
setp("cia402-drive.0.force-fault", 1)
finish_immediate_failure(0)
check_failed_home("fault during unlock wait")

c.state(linuxcnc.STATE_OFF)
c.wait_complete()
c.state(linuxcnc.STATE_ESTOP)
c.wait_complete()
if failures:
    print("%d failure(s)" % len(failures), flush=True)
    sys.exit(1)
print("success: rotary unlock and relock passed", flush=True)
sys.exit(0)
