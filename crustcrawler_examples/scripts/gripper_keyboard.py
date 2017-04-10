#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Crustcrawler SDK Gripper Example: keyboard
"""
import argparse

import rospy

import crustcrawler_interface
import crustcrawler_external_devices

def map_keyboard():
    # initialize interfaces
    print("Getting robot state... ")
    crustcrawler_gripper = crustcrawler_interface.Gripper()

    def clean_shutdown():
        print("Exiting example.")
    rospy.on_shutdown(clean_shutdown)

    def offset_dead_band(gripper, offset):
        current = gripper.parameters()['dead_zone']
        gripper.set_dead_band(current + offset)

    bindings = {
    #   key: (function, args, description)
        'r': (crustcrawler_gripper.reboot, [], "crustcrawler_gripper: reboot"),
        'c': (crustcrawler_gripper.calibrate, [], "crustcrawler_gripper: calibrate"),
        'q': (crustcrawler_gripper.close, [], "crustcrawler_gripper: close"),
        'w': (crustcrawler_gripper.open, [], "crustcrawler_gripper: open"),
        's': (crustcrawler_gripper.stop, [], "crustcrawler_gripper: stop"),
        'z': (offset_dead_band, [crustcrawler_gripper, -1.0], "crustcrawler_gripper:  decrease dead band"),
        'x': (offset_dead_band, [crustcrawler_gripper, 1.0], "crustcrawler_gripper:  increase dead band"),
    }

    done = False
    print("Enabling robot... ")
    print("Controlling grippers. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = crustcrawler_external_devices.getch()
        if c:
            if c in ['\x1b', '\x03']:
                done = True
            elif c in bindings:
                cmd = bindings[c]
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))
    # force shutdown call if caught by key handler
    rospy.signal_shutdown("Example finished.")


def main():
    """RSDK Gripper Example: Keyboard Control

    Use your dev machine's keyboard to control and configure
    Baxter's grippers.

    Run this example to command various gripper movements while
    adjusting gripper parameters, including calibration, velocity,
    and force. Uses the baxter_interface.Gripper class and the
    helper function, baxter_external_devices.getch.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_gripper_keyboard")

    map_keyboard()


if __name__ == '__main__':
    main()
