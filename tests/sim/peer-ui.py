#!/usr/bin/env python3
"""Standard-peer failures must stop active CiA sequence peers."""

import os
import subprocess
import sys
import time

import hal
import linuxcnc


h = hal.component("cia402-peer-test-ui")
h.ready()
c = linuxcnc.command()
s = linuxcnc.stat()
e = linuxcnc.error_channel()
failures = []
return_seen = False


def check(condition, message):
    if not condition:
        failures.append(message)
        print("FAIL: " + message, flush=True)


def value(name):
    return hal.get_value(name)


def setp(name, new_value):
    hal.set_p(name, str(new_value))


def sets(name, new_value):
    subprocess.run(["halcmd", "sets", name, str(new_value)], check=True)


def poll():
    global return_seen
    s.poll()
    return_seen |= value("joint.1.home-state") in (26, 27)
    while True:
        item = e.poll()
        if not item:
            break
        print("linuxcnc error %d: %s" % (item[0], item[1]), flush=True)


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


def select_custom(joint):
    setp("joint.0.request-custom-homing", int(joint == 0))
    setp("joint.1.request-custom-homing", int(joint == 1))
    time.sleep(0.1)
    poll()


def begin_home():
    c.teleop_enable(0)
    c.wait_complete()
    c.home(-1)
    c.wait_complete()


def check_peer_abort(joint, label):
    check(wait_for(lambda: value("joint.%d.cia-peer-abort" % joint)),
          label + " did not abort active CiA joint")
    check(wait_for(lambda: value("joint.%d.cia-state" % joint) == 0 and
                  value("joint.%d.home-state" % joint) == 0),
          label + " stranded CiA peer in post-abort cleanup")
    check(not value("joint.%d.cia-home" % joint),
          label + " left CiA home request asserted")
    check(value("joint.%d.cia-enable-out" % joint) ==
          value("joint.%d.cia-enable-in" % joint),
          label + " did not restore normal joint enable control")
    check(not s.homed[joint], label + " falsely marked CiA joint homed")
    check(s.task_state == linuxcnc.STATE_ON,
          label + " unexpectedly forced E-stop")


def machine_on():
    c.state(linuxcnc.STATE_ESTOP)
    c.wait_complete()
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


time.sleep(0.2)
machine_on()
setp("cia402-drive.0.homing-ticks", 500)
setp("cia402-drive.1.homing-ticks", 500)

# Standard joint 0 fails while the higher-numbered CiA joint is running.
select_custom(1)
begin_home()
check(wait_for(lambda: value("joint.0.home-state") == 7 and
              value("joint.1.cia-state") == 2),
      "standard/CiA peers did not reach active homing states")
setp("joint.0.pos-lim-sw-in", 1)
check_peer_abort(1, "forward standard-peer abort")
setp("joint.0.pos-lim-sw-in", 0)

# Reverse the joint ordering without forcing E-stop.  On 2.10, the shared
# switch rejection executes HOME_ABORT and clears every peer's sequence state.
# On 2.9 it transitions directly from HOME_START to HOME_IDLE before setting
# H[].homing.  Both must abort the lower-numbered custom peer in the same call.
machine_on()
select_custom(0)
sets("standard-switch-position", "1000")
begin_home()
check_peer_abort(0, "reversed shared-switch rejection")
sets("standard-switch-position", "-5")
check(wait_for_homing_end(), "reversed shared-switch rejection did not finish")

# Bounded-return states exist only in the 2.10 homing API.  The standard peer
# exhausts HOME_LATCH_DIST; the CiA peer must stop and join the base return.
if os.environ.get("CIA402_TEST_RETURN") == "1":
    return_seen = False
    machine_on()
    select_custom(0)
    setp("cia402-drive.0.homing-ticks", 5000)
    setp("joint.0.cia-timeout-ms", 10000)
    sets("standard-switch-position", value("joint.1.cia-pos-fb-in") - 1.0)
    begin_home()
    check(wait_for(lambda: value("joint.1.home-state") == 7 and
                  value("joint.0.cia-state") == 2),
          "bounded-return peers did not reach active homing states")
    check(wait_for(lambda: value("joint.1.home-state") == 10),
          "standard peer did not reach final switch backoff")
    sets("standard-switch-position", "1000")
    check_peer_abort(0, "bounded-return peer failure")
    check(return_seen, "standard peer never entered HOME_RETURN_START/WAIT")
    check(wait_for_homing_end(15.0), "bounded-return abort did not finish")
    sets("standard-switch-position", "-5")

if failures:
    print("%d failure(s)" % len(failures), flush=True)
    sys.exit(1)
print("success: standard peer failures aborted active CiA homing", flush=True)
sys.exit(0)
