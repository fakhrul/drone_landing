#!/usr/bin/env python

import cv2
import numpy as np

from video import Video

import time
import yaml
from dt_apriltags import Detector

import asyncio
import random
from mavsdk import System

import subprocess
import time
from collections import defaultdict
from enum import Enum
from pynput.keyboard import Listener, Key, KeyCode
from mavsdk.gimbal import GimbalMode

from pynput.keyboard import Key, Controller
from util import ImageUtil

pid = [0.5,0.5,0]
prevErrPitch = 0
prevErrRoll = 0

class Ctrl(Enum):
    (
        QUIT,
        TAKEOFF,
        LANDING,
        MOVE_LEFT,
        MOVE_RIGHT,
        MOVE_FORWARD,
        MOVE_BACKWARD,
        MOVE_UP,
        MOVE_DOWN,
        TURN_LEFT,
        TURN_RIGHT,
        ENABLE_DISABLE_KEYBOARD,
        EMERGENCY,
        ENABLE_DISABLE_PILOT
    ) = range(14)

key_esc = Key.esc
key_up = Key.up
key_down = Key.down
key_left =  Key.left
key_right = Key.right

QWERTY_CTRL_KEYS = {
    Ctrl.QUIT: key_esc,
    Ctrl.TAKEOFF: "t",
    Ctrl.LANDING: "l",
    Ctrl.MOVE_LEFT: "a",
    Ctrl.MOVE_RIGHT: "d",
    Ctrl.MOVE_FORWARD: "w",
    Ctrl.MOVE_BACKWARD: "s",
    Ctrl.MOVE_UP: key_up,
    Ctrl.MOVE_DOWN: key_down,
    Ctrl.TURN_LEFT: key_left,
    Ctrl.TURN_RIGHT: key_right,
    Ctrl.ENABLE_DISABLE_KEYBOARD: "k",
    Ctrl.EMERGENCY: "e",
    Ctrl.ENABLE_DISABLE_PILOT : "p"
}

AZERTY_CTRL_KEYS = QWERTY_CTRL_KEYS.copy()
AZERTY_CTRL_KEYS.update(
    {
        Ctrl.MOVE_LEFT: "q",
        Ctrl.MOVE_RIGHT: "d",
        Ctrl.MOVE_FORWARD: "z",
        Ctrl.MOVE_BACKWARD: "s",
    }
)


class KeyboardCtrl(Listener):
# class KeyboardCtrl():
    def __init__(self, ctrl_keys=None):
        self._ctrl_keys = self._get_ctrl_keys(ctrl_keys)
        self._key_pressed = defaultdict(lambda: False)
        self._last_action_ts = defaultdict(lambda: 0.0)
        super().__init__(on_press=self._on_press, on_release=self._on_release)
        self.start()

    def _on_press(self, key):
        if isinstance(key, KeyCode):
            self._key_pressed[key.char] = True
        elif isinstance(key, Key):
            self._key_pressed[key] = True
        if self._key_pressed[self._ctrl_keys[Ctrl.QUIT]]:
            return False
        else:
            return True

    def _on_release(self, key):
        if isinstance(key, KeyCode):
            self._key_pressed[key.char] = False
        elif isinstance(key, Key):
            self._key_pressed[key] = False
        return True

    def quit(self):
        return not self.running or self._key_pressed[self._ctrl_keys[Ctrl.QUIT]]

    def _axis(self, left_key, right_key):
        return (
            int(self._key_pressed[right_key]) - int(self._key_pressed[left_key])
        )

    def roll(self):
        return self._axis(
            self._ctrl_keys[Ctrl.MOVE_LEFT],
            self._ctrl_keys[Ctrl.MOVE_RIGHT]
        )

    def pitch(self):
        return self._axis(
            self._ctrl_keys[Ctrl.MOVE_BACKWARD],
            self._ctrl_keys[Ctrl.MOVE_FORWARD]
        )

    def yaw(self):
        return self._axis(
            self._ctrl_keys[Ctrl.TURN_LEFT],
            self._ctrl_keys[Ctrl.TURN_RIGHT]
        )

    def throttle(self):
        return self._axis(
            self._ctrl_keys[Ctrl.MOVE_DOWN],
            self._ctrl_keys[Ctrl.MOVE_UP]
        )

    def has_piloting_cmd(self):
        return (
            bool(self.roll())
            or bool(self.pitch())
            or bool(self.yaw())
            or bool(self.throttle())
        )

    def _rate_limit_cmd(self, ctrl, delay):
        now = time.time()
        if self._last_action_ts[ctrl] > (now - delay):
            return False
        elif self._key_pressed[self._ctrl_keys[ctrl]]:
            self._last_action_ts[ctrl] = now
            return True
        else:
            return False

    def takeoff(self):
        return self._rate_limit_cmd(Ctrl.TAKEOFF, 2.0)

    def landing(self):
        return self._rate_limit_cmd(Ctrl.LANDING, 2.0)

    def enable_keyboard(self):
        return self._rate_limit_cmd(Ctrl.ENABLE_DISABLE_KEYBOARD, 1)

    def enable_piloting(self):
        return self._rate_limit_cmd(Ctrl.ENABLE_DISABLE_PILOT, 1)
    
    def send_command(self, ctrl, delay):
        self._key_pressed[self._ctrl_keys[ctrl]] = True
        time.sleep(delay)
        self._key_pressed[self._ctrl_keys[ctrl]] = False


    def emergency(self):
        return self._rate_limit_cmd(Ctrl.EMERGENCY, 2.0)
    
    def _get_ctrl_keys(self, ctrl_keys):
        # Get the default ctrl keys based on the current keyboard layout:
        if ctrl_keys is None:
            ctrl_keys = QWERTY_CTRL_KEYS
            try:
                # Olympe currently only support Linux
                # and the following only works on *nix/X11...
                keyboard_variant = (
                    subprocess.check_output(
                        "setxkbmap -query | grep 'variant:'|"
                        "cut -d ':' -f2 | tr -d ' '",
                        shell=True,
                    )
                    .decode()
                    .strip()
                )
            except subprocess.CalledProcessError:
                pass
            else:
                if keyboard_variant == "azerty":
                    ctrl_keys = AZERTY_CTRL_KEYS
        return ctrl_keys


def pitchPidController(info, pid, prevErr):
    pitchErr = info
    pitchSpeed = pid[0]*pitchErr + pid[1]*(pitchErr-prevErr)
    pitchSpeed = np.clip(pitchSpeed, -0.5,0.5)
    if(pitchErr == 0):
        pitchSpeed = 0
    return pitchErr, pitchSpeed

def rollPidController(info, pid, prevErr):
    rollErr = info
    rollSpeed = pid[0]*rollErr + pid[1]*(rollErr-prevErr)
    rollSpeed = np.clip(rollSpeed, -0.5,0.5)
    if(rollErr == 0):
        rollSpeed = 0
    return rollErr, rollSpeed

def drawCorner(frame, corners, center, colors):
  # extract the bounding box (x, y)-coordinates for the AprilTag
  # # and convert each of the (x, y)-coordinate pairs to integers
    (ptA, ptB, ptC, ptD) = corners
    ptB = (int(ptB[0]), int(ptB[1]))
    ptC = (int(ptC[0]), int(ptC[1]))
    ptD = (int(ptD[0]), int(ptD[1]))
    ptA = (int(ptA[0]), int(ptA[1]))
    # draw the bounding box of the AprilTag detection
    cv2.line(frame, ptA, ptB, colors, 2)
    cv2.line(frame, ptB, ptC, colors, 2)
    cv2.line(frame, ptC, ptD, colors, 2)
    cv2.line(frame, ptD, ptA, colors, 2)
    # draw the center (x, y)-coordinates of the AprilTag
    (cX, cY) = (int(center[0]), int(center[1]))
    cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

async def manual_controls():
    """Main function to connect to the drone and input manual controls"""
    # Connect to the Simulation
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # This waits till a mavlink based drone is connected
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone with UUID: {state.uuid}")
            break

    # Checking if Global Position Estimate is ok
    async for global_lock in drone.telemetry.health():
        if global_lock.is_global_position_ok:
            print("-- Global position state is ok")
            break

    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    while not control.quit():
        
        if video.frame_available():
            frame = video.frame()
            at_detector = Detector(families='tag36h11',
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)

            cameraMatrix = np.array(parameters['sample_test']['K']).reshape((3,3))
            camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )

            gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
            tags = at_detector.detect(gray, True, camera_params, parameters['sample_test']['tag_size'])
            smallestTag = None
            for r in tags:
                if smallestTag is None:
                    smallestTag = r
                    # drawCorner(frame, r.corners,r.center, (0, 255, 0))
                else:
                    lengthSmallest = np.linalg.norm(smallestTag.corners[1]-smallestTag.corners[0])
                    lengthCurrent = np.linalg.norm(r.corners[1]-r.corners[0])
                    if lengthCurrent < lengthSmallest:
                        smallestTag = r
            

            roll = 0
            pitch = 0
            throttle = 0.5
            yaw = 0
            if smallestTag is not None:
                drawCorner(frame, smallestTag.corners, smallestTag.center, (0, 255, 0))

                height, width, channels = frame.shape
                pitch, roll = imageUtil.getMatrix(frame,width, height, int(smallestTag.center[0]), int(smallestTag.center[1]))

                tagWidth = np.linalg.norm(smallestTag.corners[1]-smallestTag.corners[0])

                cv2.putText(frame, "PITCH {}".format(pitch), (300, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, "ROLL {}".format(roll), (300, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, "THROTLE {}".format(throttle), (300, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, "YAW {}".format(yaw), (300, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.putText(frame, "WIDTH {}".format(tagWidth), (300, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                if tagWidth < 100:
                    throttle = 0.2
                    #throttle = 0.5
                    await drone.manual_control.set_manual_control_input(pitch,roll, throttle, yaw)
                else:
                    # throttle = 0.5
                    await drone.action.land()

            # else:
            #     await drone.manual_control.set_manual_control_input(pitch,roll, throttle, yaw)

            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break            

        if control.takeoff():
            print("key-takeoff")
            # Arming the drone
            print("-- Arming")
            await drone.action.arm()

            # Takeoff the vehicle
            print("-- Taking off")
            await drone.action.takeoff()
            await asyncio.sleep(5)

            print("Setting gimbal mode")
            await drone.gimbal.set_mode(GimbalMode.YAW_FOLLOW)
            await asyncio.sleep(5)

            # Move the gimbal to point at pitch -40 degrees, yaw 30 degrees
            # print("Setting pitch & yaw")
            await drone.gimbal.set_pitch_and_yaw(-90, 0)
            await asyncio.sleep(10)

            # # set the manual control input after arming
            # await drone.manual_control.set_manual_control_input(
            #     float(0), float(0), float(0.5), float(0)
            # )

            # # start manual control
            # print("-- Starting manual control")
            # await drone.manual_control.start_position_control()

        elif control.landing():
            print("key-landing")
            await drone.action.land()
        if control.has_piloting_cmd():
            print("-- piloting")
            roll = float(control.roll()) * 0.5
            pitch = float(control.pitch()) * 0.5
            throttle = float(control.throttle())
            yaw = float(control.yaw())

            print("roll=",roll)
            print("pitch=", pitch)
            print("yaw=",yaw)
            print("throtle=", throttle)

            if throttle < 0:
                throttle = 0
            await drone.manual_control.set_manual_control_input(pitch,roll, throttle, yaw)

            # await asyncio.sleep(0.1)
        # else:
        #     await drone.manual_control.set_manual_control_input(
        #         float(0), float(0), float(0.5), float(0)
        #     )        
        await asyncio.sleep(0.1)   

imageUtil = ImageUtil()
control = KeyboardCtrl()
video = Video()
with open('camera_info.yaml', 'r') as stream:
    parameters = yaml.load(stream)


keyboard = Controller()

if __name__ == '__main__':

    loop = asyncio.get_event_loop()
    loop.run_until_complete(manual_controls())
