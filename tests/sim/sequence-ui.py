#!/usr/bin/env python3
"""Sequential Home All must start every custom drive exactly once."""

import sys
import time

import hal
import linuxcnc


h = hal.component("cia402-sequence-test-ui")
h.ready()
c = linuxcnc.command()
s = linuxcnc.stat()
e = linuxcnc.error_channel()
failures = []
messages = []
attempt = None
HOME = (10.0, 20.0, 30.0)
EXPECTED_MOTOR = (110.0, 140.0, 170.0)


def check(condition, message):
    if not condition:
        failures.append(message)
        print("FAIL: " + message, flush=True)


def value(name):
    return hal.get_value(name)


def starts(joint):
    return int(value("cia402-drive.%d.homing-starts" % joint))


def poll():
    s.poll()
    while True:
        item = e.poll()
        if not item:
            break
        messages.append(item[1])
        print("linuxcnc error %d: %s" % (item[0], item[1]), flush=True)
    if attempt is None or not attempt["armed"]:
        return
    for joint in range(3):
        count = starts(joint)
        homed = bool(s.homed[joint])
        if count > attempt["before"][joint] and joint not in attempt["order"]:
            attempt["order"].append(joint)
        if (homed and not attempt["previous_homed"][joint] and
                count == attempt["before"][joint]):
            attempt["premature"].append(joint)
        attempt["previous_homed"][joint] = homed


def wait_for(condition, timeout=8.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        poll()
        if condition():
            return True
        time.sleep(0.002)
    poll()
    return condition()


def wait_for_homing_end(timeout=12.0):
    deadline = time.time() + timeout
    quiet_since = None
    while time.time() < deadline:
        poll()
        if any(j["homing"] for j in s.joint[:3]):
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
    if not wait_for(lambda: s.task_state == linuxcnc.STATE_ON):
        check(False, "machine did not reach STATE_ON")
        sys.exit(1)


def begin_home(joint):
    c.teleop_enable(0)
    c.wait_complete()
    c.home(joint)
    c.wait_complete()


def check_final_position(joint, label):
    check(abs(s.joint[joint]["input"] - HOME[joint]) < 0.02,
          "%s joint %d reached %.6f instead of HOME %.1f" %
          (label, joint, s.joint[joint]["input"], HOME[joint]))
    actual = value("joint.%d.cia-pos-fb-in" % joint)
    check(abs(actual - EXPECTED_MOTOR[joint]) < 0.02,
          "%s joint %d motor reached %.6f instead of %.1f" %
          (label, joint, actual, EXPECTED_MOTOR[joint]))


def home_all(label):
    global attempt
    poll()
    attempt = {"before": [starts(j) for j in range(3)],
               "previous_homed": [bool(s.homed[j]) for j in range(3)],
               "order": [], "premature": [], "armed": False}
    begin_home(-1)
    attempt["armed"] = True
    ended = wait_for_homing_end()
    time.sleep(0.1)
    poll()
    check(ended, label + " did not finish")
    for joint in range(3):
        check(starts(joint) == attempt["before"][joint] + 1,
              "%s joint %d starts %d -> %d, expected exactly one new start" %
              (label, joint, attempt["before"][joint], starts(joint)))
        check(s.homed[joint], "%s joint %d was not homed" % (label, joint))
        check_final_position(joint, label)
    check(attempt["order"] == [0, 1, 2],
          "%s drive start order was %s instead of [0, 1, 2]" %
          (label, attempt["order"]))
    check(not attempt["premature"],
          "%s homed joints without a fresh drive cycle: %s" %
          (label, attempt["premature"]))
    print("%s: starts=%s order=%s homed=%s" %
          (label, [starts(j) for j in range(3)], attempt["order"],
           [bool(s.homed[j]) for j in range(3)]), flush=True)
    attempt = None


def home_manually(joint):
    before = starts(joint)
    begin_home(joint)
    check(wait_for_homing_end(), "manual home joint %d did not finish" % joint)
    time.sleep(0.05)
    poll()
    check(starts(joint) == before + 1,
          "manual home joint %d did not start its drive exactly once" % joint)
    check(s.homed[joint], "manual home joint %d was not homed" % joint)
    check_final_position(joint, "manual home")


machine_on()
home_all("fresh Home All with stale mode-entry status")
hal.set_p("cia402-drive.2.scenario", "0")

home_manually(0)
home_all("Home All after manual joint 0")

home_manually(0)
home_manually(1)
home_all("Home All after manual joints 0 and 1")

home_manually(2)
home_all("Home All after manual joint 2")
home_all("repeated Home All")

check(not any("following error" in m.lower() for m in messages),
      "sequential Home All caused a LinuxCNC following error")

c.state(linuxcnc.STATE_OFF)
c.wait_complete()
c.state(linuxcnc.STATE_ESTOP)
c.wait_complete()
if failures:
    print("%d failure(s)" % len(failures), flush=True)
    sys.exit(1)
print("success: sequential and partially pre-homed Home All passed", flush=True)
sys.exit(0)
