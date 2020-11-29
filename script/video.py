
import cv2
import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst

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