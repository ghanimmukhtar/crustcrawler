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

import re
import sys
import time

from copy import deepcopy
from math import fabs

from json import (
    JSONDecoder,
    JSONEncoder,
)

from crustcrawler_core_msgs.msg import(
    EndEffectorCommand,
    EndEffectorState,
)

import rospy

import crustcrawler_dataflow

from crustcrawler_interface import settings


class Gripper(object):
    """
    Interface class for a gripper on the Baxter Research Robot.
    """
    def __init__(self):
        self.name = 'gripper'
        self._cmd_sender = rospy.get_name() + '_%s'
        self._cmd_sequence = 0

        ns = 'crustcrawler/end_effector/' + self.name + "/"

        self._state = None
        self.on_gripping_changed = crustcrawler_dataflow.Signal()
        self.on_moving_changed = crustcrawler_dataflow.Signal()

        self._parameters = dict()

        self._cmd_pub = rospy.Publisher(ns + 'command', EndEffectorCommand,
            queue_size=10)

        self._state_pub = rospy.Publisher(ns + 'set_state',
                                          EndEffectorState,
                                          latch=True,
                                          queue_size=10
                                          )

        self._state_sub = rospy.Subscriber(ns + 'state',
                                           EndEffectorState,
                                           self._on_gripper_state
                                           )

        # Wait for the gripper state message to be populated
        crustcrawler_dataflow.wait_for(
                          lambda: not self._state is None,
                          timeout=5.0,
                          timeout_msg=("Failed to get state from %s" %
                                       (ns + 'state',))
                          )

    def _on_gripper_state(self, state):
        old_state = self._state
        self._state = deepcopy(state)
        if old_state is not None and old_state.gripping != state.gripping:
            self.on_gripping_changed(state.gripping == True)
        if old_state is not None and old_state.moving != state.moving:
            self.on_moving_changed(state.moving == True)

    def _inc_cmd_sequence(self):
        # manage roll over with safe value (maxint)
        self._cmd_sequence = (self._cmd_sequence % 0x7FFFFFFF) + 1
        return self._cmd_sequence

    def _clip(self, val):
        return max(min(val, 100.0), 0.0)

    def command(self, cmd, block=False, test=lambda: True,
                 timeout=0.0, args=None):
        """
        Raw command call to directly control gripper.

        @type cmd: str
        @param cmd: string of known gripper commands
        @type block: bool
        @param block: command is blocking or non-blocking [False]
        @type test: func
        @param test: test function for command validation
        @type timeout: float
        @param timeout: timeout in seconds for command evaluation
        @type args: dict({str:float})
        @param args: dictionary of parameter:value
        """
        ee_cmd = EndEffectorCommand()
        ee_cmd.id = self.hardware_id()
        ee_cmd.command = cmd
        ee_cmd.sender = self._cmd_sender % (cmd,)
        ee_cmd.sequence = self._inc_cmd_sequence()
        ee_cmd.args = ''
        if args != None:
            ee_cmd.args = JSONEncoder().encode(args)
        seq_test = lambda: (self._state.command_sender == ee_cmd.sender and
                            (self._state.command_sequence == ee_cmd.sequence
                             or self._state.command_sequence == 0))
        self._cmd_pub.publish(ee_cmd)
        if block:
            finish_time = rospy.get_time() + timeout
            cmd_seq = crustcrawler_dataflow.wait_for(
                          test=seq_test,
                          timeout=timeout,
                          raise_on_error=False,
                          body=lambda: self._cmd_pub.publish(ee_cmd)
                      )
            if not cmd_seq:
                seq_msg = (("Timed out on gripper command acknowledgement for"
                           " %s:%s") % (self.name, ee_cmd.command))
                rospy.logdebug(seq_msg)
            time_remain = max(0.5, finish_time - rospy.get_time())
            return crustcrawler_dataflow.wait_for(
                       test=test,
                       timeout=time_remain,
                       raise_on_error=False,
                       body=lambda: self._cmd_pub.publish(ee_cmd)
                   )
        else:
            return True

    def reset_custom_state(self, timeout=2.0):
        """
        Resets default state for custom grippers

        @return: True if custom gripper state reset successfully
        @rtype: bool
        """
        state_true = EndEffectorState.STATE_TRUE
        state_unknown = EndEffectorState.STATE_UNKNOWN
        # Create default state message
        state_msg = EndEffectorState()
        for idx, attr in enumerate(state_msg.__slots__):
            if 'int' in state_msg._slot_types[idx]:
                setattr(state_msg, attr, state_unknown)
        setattr(state_msg, 'enabled', state_true)
        self._state_pub.publish(state_msg)

        # Verify state reset successfully
        test = lambda: (self._state.enabled == state_true and
                        self._state.calibrated == state_unknown and
                        self._state.ready == state_unknown and
                        self._state.position == 0.0
                        )
        return crustcrawler_dataflow.wait_for(
                   test=test,
                   timeout=timeout,
                   raise_on_error=False,
                   body=lambda: self._state_pub.publish(state_msg)
               )

    def reset(self, block=True, timeout=2.0):
        """
        Resets the gripper state removing any errors.

        @type timeout: float
        @param timeout: timeout in seconds for reset success
        @type block: bool
        @param block: command is blocking or non-blocking [False]
        """
        cmd = EndEffectorCommand.CMD_RESET
        return self.command(
                            cmd,
                            block,
                            test=lambda: (self._state.error == False and
                                          self._state.ready == True),
                            timeout=timeout,
                            )

    def _cmd_reboot(self, block=True, timeout=5.0):
        """
        Power cycle the gripper, removing calibration information.

        Basic call to the gripper reboot command. Waits for gripper to return
        ready state but does not clear errors that could occur during boot.
        Highly recommended to use the clean reboot() command instead.

        @type timeout: float
        @param timeout: timeout in seconds for reboot success
        @type block: bool
        @param block: command is blocking or non-blocking [False]
        """
        cmd = EndEffectorCommand.CMD_REBOOT
        success = self.command(
                      cmd,
                      block,
                      test=lambda: (self._state.enabled == True and
                                    self._state.ready == True),
                      timeout=timeout
        )
        rospy.sleep(1.0)  # Allow extra time for reboot to complete
        self.set_parameters(defaults=True)
        return success

    def reboot(self, timeout=5.0, delay_check=0.1):
        """
        "Clean" reboot of gripper, removes calibration and errors.

        Robust version of gripper reboot command; recommended to use this
        function for rebooting grippers.

        Calls the basic reboot gripper command to power cycle the gripper
        (_cmd_reboot()) and then checks for errors after reboot, calling
        reset to clear errors if needed.

        @type timeout: float
        @param timeout: timeouts in seconds for reboot & reset
        @type delay_check: float
        @param delay_check: seconds after reboot before error check
        """
        self._cmd_reboot(block=True, timeout=timeout)
        rospy.sleep(delay_check)
        if self.error():
            if not self.reset(block=True, timeout=timeout):
                rospy.logerr("Failed to reset gripper error after reboot.")
                return False
        return True

    def clear_calibration(self, block=True, timeout=2.0):
        """
        Clear calibration information from gripper.

        Allows (and requires) new gripper calibration to be run.

        @type timeout: float
        @param timeout: timeout in seconds for success
        @type block: bool
        @param block: command is blocking or non-blocking [False]
        """
        cmd = EndEffectorCommand.CMD_CLEAR_CALIBRATION
        return self.command(
                   cmd,
                   block,
                   test=lambda: (self._state.calibrated == False and
                                 self._state.ready == True),
                   timeout=timeout
        )

    def calibrate(self, block=True, timeout=5.0):
        """
        Calibrate the gripper setting maximum and minimum travel distance.

        @type timeout: float
        @param timeout: timeout in seconds for calibration success
        @type block: bool
        @param block: command is blocking or non-blocking [False]
        @rtype: bool
        @return: Returns True if calibration succeeds.
        """
        # clear any previous calibration and any current errors
        if self.calibrated():
            self.clear_calibration()
        if self.error():
            self.reset(block=block)

        cmd = EndEffectorCommand.CMD_CALIBRATE
        success = self.command(
                      cmd,
                      block,
                      test=lambda: (self._state.calibrated == True and
                                    self._state.ready == True),
                      timeout=timeout
                      )
        return success

    def stop(self, block=True, timeout=5.0):
        """
        Stop the gripper at the current position and apply holding force.

        @type timeout: float
        @param timeout: timeout in seconds for stop success
        @type block: bool
        @param block: command is blocking or non-blocking [False]
        """
        cmd = EndEffectorCommand.CMD_STOP
        stop_test = lambda: self._state.moving == False
        return self.command(
                            cmd,
                            block,
                            test=stop_test,
                            timeout=timeout,
                            )

    def command_position(self, position, block=False, timeout=5.0):
        """
        Command the gripper position movement.

        @type position: float
        @param position: in % 0=close 100=open

        From minimum/closed (0.0) to maximum/open (100.0)
        """
        if self._state.calibrated != True:
            msg = "Unable to command %s position until calibrated" % self.name
            rospy.logwarn(msg)
            return False

        cmd = EndEffectorCommand.CMD_GO
        arguments = {"position": self._clip(position)}
        cmd_test = lambda: (self._state.gripping == True)
        return self.command(
                            cmd,
                            block,
                            test=cmd_test,
                            timeout=timeout,
                            args=arguments
                            )

    def open(self, block=False, timeout=5.0):
        """
        Commands maximum gripper position.

        @type block: bool
        @param block: open command is blocking or non-blocking [False]
        @type timeout: float
        @param timeout: timeout in seconds for open command success
        """
        return self.command_position(position=100.0, block=block,
                                     timeout=timeout)

    def close(self, block=False, timeout=5.0):
        """
        Commands minimum gripper position.

        @type block: bool
        @param block: close command is blocking or non-blocking [False]
        @type timeout: float
        @param timeout: timeout in seconds for close command success
        """
        return self.command_position(position=0.0, block=block,
                                     timeout=timeout)

    def calibrated(self):
        """
        Returns bool describing gripper calibration state.
        (0:Not Calibrated, 1:Calibrated)

        @rtype: bool
        @return: False if Not Calibrated, True if Calibrated.
        Grippers that cannot calibrate should return True
        (i.e. "Always calibrated").
        """
        return self._state.calibrated == True

    def ready(self):
        """
        Returns bool describing if the gripper ready, i.e. is calibrated, not
        busy (as in resetting or rebooting), and not moving.

        @rtype: bool
        @return: True if gripper is not busy
        """
        return self._state.ready == True

    def moving(self):
        """
        Returns bool describing if the gripper is in motion

        @rtype: bool
        @return: True if gripper is in motion
        """
        return self._state.moving == True

    def gripping(self):
        """
        Returns bool describing if the position move has been preempted by a
        position command exceeding the moving_force threshold denoting a grasp.

        @rtype: bool
        """
        return self._state.gripping == True

    def missed(self):
        """
        Returns bool describing if the position move has completed without
        exceeding the moving_force threshold denoting a grasp

        @rtype: bool
        """
        return self._state.missed == True

    def error(self):
        """
        Returns bool describing if the gripper is in an error state.

        Error states can be caused by over/undervoltage, over/under current,
        motor faults, etc.

        Errors can be cleared with a gripper reset. If persistent please
        contact Rethink Robotics for further debugging.

        @rtype: bool
        """
        return self._state.error == True

    def position(self):
        """
        Returns the current gripper position as a ratio (0-100) of the total
        gripper travel.

        @rtype: float
        """
        return deepcopy(self._state.position)

    def force(self):
        """
        Returns the current measured gripper force as a ratio (0-100) of the
        total force applicable.

        @rtype: float
        """
        return deepcopy(self._state.force)

    def has_force(self):
        """
        Returns bool describing if the gripper is capable of force control.

        @rtype: bool
        """
        return self._prop.controls_force == True

    def has_position(self):
        """
        Returns bool describing if the gripper is capable of position control.

        @rtype: bool
        """
        return self._prop.controls_position == True

    def hardware_id(self):
        """
        Returns unique hardware id number. This is required for sending
        commands to the gripper.

        @rtype: int
        """
        return deepcopy(self._state.id)

    def hardware_name(self):
        """
        Returns string describing the gripper hardware.

        @rtype: str
        """
        return deepcopy(self._prop.product)
