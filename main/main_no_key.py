#!/usr/bin/env python

from mavsdk import System
import cv2
import numpy as np

import cv2.aruco as aruco
import sys, time, math

# from video import Video

import time
import yaml
# from dt_apriltags import Detector

import asyncio

import random

import subprocess
import time
from collections import defaultdict
from enum import Enum
# from pynput.keyboard import Listener, Key, KeyCode
from mavsdk.gimbal import GimbalMode

# from pynput.keyboard import Key, Controller
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

key_esc = ''
key_up = ''
key_down = ''
key_left =  ''
key_right = ''

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


class KeyboardCtrl():
# class KeyboardCtrl():
    def __init__(self, ctrl_keys=None):
        self._ctrl_keys = self._get_ctrl_keys(ctrl_keys)
        self._key_pressed = defaultdict(lambda: False)
        self._last_action_ts = defaultdict(lambda: 0.0)
        # super().__init__(on_press=self._on_press, on_release=self._on_release)
        # self.start()

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

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

async def manual_controls():
    """Main function to connect to the drone and input manual controls"""
    print("Application Started. Live view is enable? {}".format(isEnableDroneStream))
    # Connect to the Simulation
    if isEnableDroneControl:
        drone = System()
        # await drone.connect(system_address="udp://:14540")
        await drone.connect(system_address="serial:///dev/ttyACM0")

        # This waits till a mavlink based drone is connected
        print("Waiting for drone to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone with UUID: {state.uuid}")
                break

        # print("Waiting for drone to connect...")
        # async for state in drone.core.connection_state():
        #     if state.is_connected:
        #         print(f"Drone discovered with UUID: {state.uuid}")
        #         break

        info = await drone.info.get_version()
        print(info)

        # Checking if Global Position Estimate is ok
        async for global_lock in drone.telemetry.health():
            if global_lock.is_global_position_ok:
                print("-- Global position state is ok")
                break

        # set the manual control input after arming
        await drone.manual_control.set_manual_control_input(
            float(0), float(0), float(0.5), float(0)
        )

    while True:

        ret, frame = cap.read()
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
        smallestTag = None
        smallestTagCenter = None

        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                                cameraMatrix=camera_matrix, distCoeff=camera_distortion)

        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                
                if isEnableDroneStream:
                    # draw the bounding box of the ArUCo detection
                    cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the
                # ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                if isEnableDroneStream:
                    cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                    # draw the ArUco marker ID on the frame
                    cv2.putText(frame, str(markerID),
                        (topLeft[0], topLeft[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
                
                if smallestTag is None:
                    smallestTag = corners
                    smallestTagCenter = (cX, cY)
                else:
                    lengthSmallest = np.linalg.norm(smallestTag[1]-smallestTag[0])
                    lengthCurrent = np.linalg.norm(corners[1]-corners[0])
                    if lengthCurrent < lengthSmallest:
                        smallestTag = corners
                        smallestTagCenter =(cX, cY)
        
        if smallestTag is not None:
            (topLeft, topRight, bottomRight, bottomLeft) = smallestTag
            topLeft = (int(topLeft[0]), int(topLeft[1]))
 
            if isEnableDroneStream:
                cv2.putText(frame, str('smallest'),
                    (topLeft[0], topLeft[1]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 255), 2)

        roll = 0
        pitch = 0
        throttle = 0.5
        yaw = 0
        if smallestTag is not None:
 
            height, width, channels = frame.shape
            pitch, roll = imageUtil.getMatrix(frame,width, height, int(smallestTagCenter[0]), int(smallestTagCenter[1]))

            (topLeft, topRight, bottomRight, bottomLeft) = smallestTag
            tagWidth = np.linalg.norm(topLeft[0]-topRight[0])
            # tagWidth = np.linalg.norm(smallestTagCenter[1]-smallestTagCenter[0])

            cv2.putText(frame, "PITCH {}".format(pitch), (300, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, "ROLL {}".format(roll), (300, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, "THROTLE {}".format(throttle), (300, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, "YAW {}".format(yaw), (300, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.putText(frame, "WIDTH {}".format(tagWidth), (300, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            print("FOUND TAG    >   PITCH   {}, ROLL    {}, THROTLE {}, YAW     {}, WIDTH   {}".format(pitch, roll, throttle, yaw, tagWidth))


            if isEnableDroneControl:
                if tagWidth < 200:
                    throttle = 0.2
                    #throttle = 0.5
                    print("INSTRUCTION MOVE")
                    await drone.manual_control.set_manual_control_input(pitch,roll, throttle, yaw)
                else:
                    # throttle = 0.5
                    print("INSTRUCTION LAND")
                    await drone.action.land()


        if isEnableDroneStream:
            cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break    

        if control.takeoff():
            print("key      >   takeoff")
            if isEnableDroneControl:
                print("drone    >   arming")
                await drone.action.arm()

                print("drone    >   takeoff")
                await drone.action.takeoff()
                await asyncio.sleep(5)

                # print("Setting gimbal mode")
                # await drone.gimbal.set_mode(GimbalMode.YAW_FOLLOW)
                # await asyncio.sleep(5)

                # print("Setting pitch & yaw")
                # await drone.gimbal.set_pitch_and_yaw(-90, 0)
                # await asyncio.sleep(10)

            # # set the manual control input after arming
            # await drone.manual_control.set_manual_control_input(
            #     float(0), float(0), float(0.5), float(0)
            # )

            # # start manual control
            # print("-- Starting manual control")
            # await drone.manual_control.start_position_control()

        elif control.landing():
            print("key      >   landing")
            if isEnableDroneControl:
                print("drone    >   landing")
                await drone.action.land()
        if control.has_piloting_cmd():
            print("key      >   piloting")
            roll = float(control.roll()) * 0.5
            pitch = float(control.pitch()) * 0.5
            throttle = float(control.throttle())
            yaw = float(control.yaw())

            print("piloting >   roll    {}  pitch   {}  yaw     {}  throtle {}".format(roll, pitch, yaw, throttle))
            # print("roll=",roll)
            # print("pitch=", pitch)
            # print("yaw=",yaw)
            # print("throtle=", throttle)

            if throttle < 0:
                throttle = 0
            if isEnableDroneControl:
               await drone.manual_control.set_manual_control_input(pitch,roll, throttle, yaw)

            # await asyncio.sleep(0.1)
        # else:
        #     await drone.manual_control.set_manual_control_input(
        #         float(0), float(0), float(0.5), float(0)
        #     )        
        await asyncio.sleep(0.1)   

imageUtil = ImageUtil()
control = KeyboardCtrl()
# video = Video()
# with open('camera_info.yaml', 'r') as stream:
#     parameters = yaml.load(stream)

#--- Define Tag
id_to_find  = 72
marker_size  = 10 #- [cm]

#--- Get the camera calibration path
calib_path  = ""
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters  = aruco.DetectorParameters_create()


#--- Capture the videocamera (this may also be a video or a picture)
cap = cv2.VideoCapture(0)
#-- Set the camera size as the one it was calibrated with
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

# keyboard = Controller()

isEnableDroneControl = False
isEnableDroneStream = False

if __name__ == '__main__':

    loop = asyncio.get_event_loop()
    loop.run_until_complete(manual_controls())
