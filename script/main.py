#!/usr/bin/env python

import cv2
import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst

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

# key_esc = "1"
# key_up = "2"
# key_down = "3"
# key_left =  "4"
# key_right = "5"

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






class Video():
    def __init__(self, port=5600):

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None
        self.frameHeight = 0
        self.frameWidth = 0
        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(self, sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        self.frameHeight =caps.get_structure(0).get_value('height')
        self.frameWidth = caps.get_structure(0).get_value('width')
        array = np.ndarray(
            (
                self.frameHeight,
                self.frameWidth,
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)

        return array

    def frame(self):
        """ Get Frame

        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available

        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)


    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(self, sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK

    def displayPartition(self, drone, img, targetX, targetY):
        # draw vertical
        line = 3
        for x in range(line):
            cv2.line(img,(int(self.frameWidth/line) * x,0),(int(self.frameWidth/line) * x,self.frameHeight),(255,255,0),1)
            
        for x in range(line):
            cv2.line(img, (0,int(self.frameHeight / line) * x), (self.frameWidth,int(self.frameHeight / line) * x), (255, 255, 0), 1)

        center_x = int(self.frameWidth/2)
        center_y = int(self.frameHeight/2)
        cv2.circle(img,(center_x,center_y),2,(0,0,255),2)
        cv2.putText(img, "({},{})".format(center_x, center_y), (center_x, center_y + 20), cv2.FONT_HERSHEY_COMPLEX, 0.5,(0, 0, 255), 1)


        roll = 0
        pitch = 0
        throttle = 0
        yaw = 0

        if targetX > 0 and targetY > 0:


            cv2.putText(img, "({},{})".format(targetX, targetY), (targetX, targetY + 20), cv2.FONT_HERSHEY_COMPLEX, 0.5,(0, 0, 255), 1)
            if targetY < int(self.frameHeight/3):
                cv2.putText(img, "NORTH", (20, 50), cv2.FONT_HERSHEY_COMPLEX,0.8,(0, 0, 255), 3)
                pitch = 0.5    
            if targetY > int(self.frameHeight/3) * 2:
                cv2.putText(img, "SOUTH", (20, 50), cv2.FONT_HERSHEY_COMPLEX,0.8,(0, 0, 255), 3)
                pitch = -0.5    

            if targetX < int(self.frameWidth/3):
                cv2.putText(img, "WEST", (20, 80), cv2.FONT_HERSHEY_COMPLEX,0.8,(0, 0, 255), 3)
                roll = -0.2    

            if targetX > int(self.frameWidth/3) * 2:
                cv2.putText(img, "EAST", (20, 80), cv2.FONT_HERSHEY_COMPLEX,0.8,(0, 0, 255), 3)
                roll = 0.2    
        return roll, pitch, throttle, yaw


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
            for r in tags:
                # extract the bounding box (x, y)-coordinates for the AprilTag
                # and convert each of the (x, y)-coordinate pairs to integers
                (ptA, ptB, ptC, ptD) = r.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                # draw the bounding box of the AprilTag detection
                cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
                cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
                cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
                cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
                # draw the center (x, y)-coordinates of the AprilTag
                (cX, cY) = (int(r.center[0]), int(r.center[1]))
                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

            if len(tags) > 0:
                roll, pitch, throttle, yaw = video.displayPartition(drone, frame, int(tags[0].center[0]), int(tags[0].center[1]))
                cv2.putText(frame, "PITCH {}".format(pitch), (300, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, "ROLL {}".format(roll), (300, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, "THROTLE {}".format(throttle), (300, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, "YAW {}".format(yaw), (300, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                throttle = 0.5
                await drone.manual_control.set_manual_control_input(pitch,roll, throttle, yaw)

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

control = KeyboardCtrl()
video = Video()
with open('camera_info.yaml', 'r') as stream:
    parameters = yaml.load(stream)


keyboard = Controller()

if __name__ == '__main__':

    loop = asyncio.get_event_loop()
    loop.run_until_complete(manual_controls())


    while True:
        # Wait for the next frame
        if not video.frame_available():
            continue

        frame = video.frame()

          ## start
        # frame = cv2.imread('/home/prsb/catkin_ws/src/fan_apriltag/src/apriltag/images/mapping_feb_2014/IMG_1368.JPG',
        #                   cv2.IMREAD_COLOR)  # road.png is the filename
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
        # result = detector.detect(gray)

        #     # Now we loop through each detected tag
        # for i in range(len(result)):
        #     pts = np.array(result[i].corners, np.int32)
        #     pts = pts.reshape((-1, 1, 2))
        #     frame = cv2.polylines(frame, [pts], True, (0, 255, 255), 10)
            
        #     font = cv2.FONT_HERSHEY_SIMPLEX
        #     cv2.putText(frame, str(result[i].tag_id), tuple(np.array(result[i].center, np.int32)), font, 3, (255, 255, 255), 5, cv2.LINE_AA)

        # ## end
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
        print(tags)
        for r in tags:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
            cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
            cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
            cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
            # draw the tag family on the image
            tagFamily = r.tag_family.decode("utf-8")
            cv2.putText(frame, tagFamily, (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("[INFO] tag family: {}".format(tagFamily))

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
