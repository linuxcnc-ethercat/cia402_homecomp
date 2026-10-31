#!/usr/bin/env python3
"""Verify absolute-encoder homing stays on LinuxCNC's base path."""

import sys
import time

import hal
import linuxcnc


h = hal.component("cia402-absolute-test-ui")
h.ready()
c = linuxcnc.command()
s = linuxcnc.stat()
e = linuxcnc.error_channel()
failures = []


def check(condition, message):
    if not condition:
        failures.append(message)
        print("FAIL: " + message, flush=True)


def value(name):
    return hal.get_value(name)


def poll():
    s.poll()
    while e.poll():
        pass


def wait_for(condition, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        poll()
        if condition():
            return True
        time.sleep(0.002)
    poll()
    return condition()


time.sleep(0.2)
c.state(linuxcnc.STATE_ESTOP_RESET)
c.wait_complete()
wait_for(lambda: s.task_state == linuxcnc.STATE_ESTOP_RESET, 2.0)
c.state(linuxcnc.STATE_ON)
c.wait_complete()
c.mode(linuxcnc.MODE_MANUAL)
c.wait_complete()
if not wait_for(lambda: s.task_state == linuxcnc.STATE_ON):
    print("FAIL: machine did not reach STATE_ON", flush=True)
    sys.exit(1)

starts = value("cia402-drive.0.homing-starts")
actual = value("joint.0.cia-pos-fb-in")
c.teleop_enable(0)
c.wait_complete()
c.home(0)
c.wait_complete()
check(wait_for(lambda: s.homed[0] and not s.joint[0]["homing"]),
      "base absolute-encoder homing did not finish")
time.sleep(0.1)
poll()
check(abs(s.joint[0]["input"] - 5.0) < 0.02,
      "absolute reference is %.6f instead of HOME_OFFSET 5.0" %
      s.joint[0]["input"])
check(abs(value("joint.0.cia-pos-fb-in") - actual) < 0.02,
      "absolute homing caused drive-owned motion")
check(value("cia402-drive.0.homing-starts") == starts,
      "absolute homing started CiA drive homing")
check(value("joint.0.cia-state") == 0 and not value("joint.0.cia-home"),
      "absolute homing used custom drive-homing state")
check(not value("joint.0.unlock"),
      "absolute homing incorrectly requested rotary unlock")

# HOME_ABSOLUTE_ENCODER=2 implies HOME_NO_REHOME and HOME_NO_FINAL_MOVE.
c.home(0)
c.wait_complete()
time.sleep(0.3)
poll()
check(value("cia402-drive.0.homing-starts") == starts,
      "absolute no-rehome request started drive homing")
check(s.homed[0], "absolute no-rehome request cleared homed state")
check(abs(value("joint.0.cia-pos-fb-in") - actual) < 0.02,
      "absolute no-rehome/no-final request moved the motor")

c.state(linuxcnc.STATE_OFF)
c.wait_complete()
c.state(linuxcnc.STATE_ESTOP)
c.wait_complete()
if failures:
    print("%d failure(s)" % len(failures), flush=True)
    sys.exit(1)
print("success: absolute encoder used base no-final/no-rehome homing", flush=True)
sys.exit(0)
