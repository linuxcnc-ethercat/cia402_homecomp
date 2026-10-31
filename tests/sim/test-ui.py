#!/usr/bin/env python3
"""Headless integration tests for drive-controlled and standard homing."""

import sys
import time

import hal
import linuxcnc


h = hal.component("cia402-test-ui")
h.ready()

c = linuxcnc.command()
s = linuxcnc.stat()
e = linuxcnc.error_channel()

failures = []
messages = []
monitor = {"external_motion": 0.0, "held_error": 0.0, "mode6": False}


def fail(message):
    failures.append(message)
    print("FAIL: " + message, flush=True)


def check(condition, message):
    if not condition:
        fail(message)


def value(name):
    return hal.get_value(name)


def setp(name, new_value):
    hal.set_p(name, str(new_value))


def drain_errors():
    while True:
        item = e.poll()
        if not item:
            break
        messages.append(item[1])
        print("linuxcnc error %d: %s" % (item[0], item[1]), flush=True)


def poll():
    s.poll()
    drain_errors()
    if any(j["homing"] for j in s.joint[:2]):
        command = value("joint.0.cia-pos-cmd")
        feedback = value("joint.0.cia-pos-fb-out")
        actual = value("joint.0.cia-pos-fb-in")
        monitor["held_error"] = max(monitor["held_error"], abs(command - feedback))
        monitor["external_motion"] = max(monitor["external_motion"], abs(actual))
        monitor["mode6"] |= value("joint.0.cia-opmode-display") == 6


def wait_for(condition, timeout=8.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        poll()
        if condition():
            return True
        time.sleep(0.002)
    poll()
    return condition()


def wait_for_homing_end(timeout):
    """Wait for a stable idle gap, not a one-poll gap between sequences."""
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
        fail("machine did not reach STATE_ON (state=%d estop=%d enabled=%d)" %
             (s.task_state, s.estop, s.enabled))
        sys.exit(1)


def begin_home(joint):
    c.teleop_enable(0)
    c.wait_complete()
    c.home(joint)
    c.wait_complete()
    return wait_for(lambda: any(j["homing"] for j in s.joint[:2]) or
                    value("joint.0.cia-opmode-display") == 6, 3.0)


def finish_home(joint, timeout=10.0):
    monitor.update(external_motion=0.0, held_error=0.0, mode6=False)
    started = begin_home(joint)
    check(started, "homing did not start for joint %d" % joint)
    if not started:
        print("start debug: task-state=%d home-state=%d cia-state=%d stat-fault=%d "
              "home-pin=%d enable=%d request=%d" %
              (s.task_state, value("joint.0.home-state"), value("joint.0.cia-state"),
               value("cia402.0.stat-fault"), value("joint.0.cia-home"),
               value("joint.0.cia-enable-out"),
               value("joint.0.request-custom-homing")), flush=True)
    ended = wait_for_homing_end(timeout)
    time.sleep(0.1)
    poll()
    check(ended, "homing did not finish for joint %d" % joint)
    print("home(%d): started=%d ended=%d homed=%d input=%.3f actual=%.3f "
          "home-pin=%d enable=%d mode=%d" %
          (joint, started, ended, s.homed[0], s.joint[0]["input"],
           value("joint.0.cia-pos-fb-in"), value("joint.0.cia-home"),
           value("joint.0.cia-enable-out"), value("joint.0.cia-opmode-display")),
          flush=True)
    return ended


def unhome(joint):
    c.teleop_enable(0)
    c.wait_complete()
    c.unhome(joint)
    c.wait_complete()
    check(wait_for(lambda: not s.homed[joint]), "joint %d did not unhome" % joint)


def check_no_following_error(label):
    check(not any("following error" in m.lower() for m in messages),
          "%s caused a LinuxCNC following error" % label)


def check_failed_home(label):
    check(not s.homed[0], "%s falsely marked joint 0 homed" % label)
    check(not value("joint.0.cia-home"), "%s left cia402.home asserted" % label)
    check(not value("joint.0.cia-enable-out"),
          "%s did not revoke drive enable" % label)
    check_no_following_error(label)


def reset_drive_fault():
    setp("cia402.0.fault-reset", 1)
    check(wait_for(lambda: not value("cia402.0.stat-fault")),
          "simulated drive fault did not reset")
    setp("cia402.0.fault-reset", 0)
    time.sleep(0.05)
    poll()


machine_on()
setp("cia402-drive.0.scenario", 0)

# Drive moves 100 mm while LinuxCNC's command stays fixed.  Ordinary ferror
# limits are only 0.10/0.25 mm, so feedback gating is essential here.
finish_home(0)
check(s.homed[0], "successful drive homing did not mark joint 0 homed")
check(monitor["mode6"], "drive never acknowledged operation mode 6")
check(monitor["external_motion"] >= 95.0,
      "virtual drive did not execute large autonomous motion")
check(monitor["held_error"] < 0.02,
      "feedback was not synchronized during drive homing")
check(abs(s.joint[0]["input"] - 25.0) < 0.02,
      "HOME position is %.6f, expected 25.0" % s.joint[0]["input"])
check(abs(value("joint.0.cia-pos-fb-in") - 120.0) < 0.02,
      "CSP final motor feedback is %.6f, expected 120.0" % value("joint.0.cia-pos-fb-in"))
check(value("joint.0.cia-opmode-display") == 8,
      "drive did not return to CSP after homing")
check(value("joint.0.cia-enable-out"), "drive enable did not recover after success")
check_no_following_error("successful homing")
print("success scenario: external motion %.3f mm, HOME %.3f mm" %
      (monitor["external_motion"], s.joint[0]["input"]), flush=True)

unhome(0)
setp("cia402-drive.0.scenario", 1)
finish_home(0)
check_failed_home("homing error with target-reached")

setp("cia402-drive.0.scenario", 2)
finish_home(0)
check_failed_home("drive fault")
reset_drive_fault()

setp("cia402-drive.0.scenario", 3)
setp("joint.0.cia-timeout-ms", 100)
finish_home(0)
check_failed_home("stalled drive timeout")
setp("joint.0.cia-timeout-ms", 1000)

setp("cia402-drive.0.scenario", 0)
setp("cia402-drive.0.homing-ticks", 500)
check(begin_home(0), "cancellation scenario did not start")
check(wait_for(lambda: value("joint.0.cia-opmode-display") == 6),
      "cancellation scenario did not enter mode 6")
c.abort()
c.wait_complete()
check(wait_for(lambda: not any(j["homing"] for j in s.joint[:2])),
      "cancellation did not stop homing")
time.sleep(0.1)
poll()
check_failed_home("user cancellation")

setp("cia402-drive.0.homing-ticks", 40)
finish_home(0)
check(s.homed[0], "explicit retry after cancellation did not home")
unhome(0)
finish_home(0)
check(s.homed[0], "rehome from a nonzero position did not home")
check(abs(s.joint[0]["input"] - 25.0) < 0.02,
      "rehome did not restore HOME")
check_no_following_error("rehome")

# Both sequence groups run together; joint 1 must use base homing while
# joint 0 repeats drive-controlled homing.
unhome(0)
unhome(1)
finish_home(-1, timeout=15.0)
check(s.homed[0] and s.homed[1], "mixed CiA/standard homing did not home both")
check(abs(s.joint[1]["input"] - 20.0) < 0.02,
      "standard joint HOME is %.6f, expected 20.0" % s.joint[1]["input"])
check_no_following_error("mixed homing")

c.state(linuxcnc.STATE_OFF)
c.wait_complete()
c.state(linuxcnc.STATE_ESTOP)
c.wait_complete()

if failures:
    print("%d failure(s)" % len(failures), flush=True)
    sys.exit(1)
print("success: all CiA 402 homing simulation scenarios passed", flush=True)
sys.exit(0)
