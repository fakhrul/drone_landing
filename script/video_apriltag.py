#!/usr/bin/env python

import cv2
import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst

import time
import yaml
from dt_apriltags import Detector
from util import ImageUtil

cap = cv2.VideoCapture(0)
tag_size = 0.055 #in meters

frameWidth = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frameHeight = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)
deadZone =0

imageUtil = ImageUtil()

def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)

    # draw ground floor in green
    img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)

    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)

    # draw top layer in red color
    img = cv2.drawContours(img, [imgpts[4:]],-1,(0,0,255),3)

    return img


# def stackImages(scale,imgArray):
#     rows = len(imgArray)
#     cols = len(imgArray[0])
#     rowsAvailable = isinstance(imgArray[0], list)
#     width = imgArray[0][0].shape[1]
#     height = imgArray[0][0].shape[0]
#     if rowsAvailable:
#         for x in range ( 0, rows):
#             for y in range(0, cols):
#                 if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
#                     imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
#                 else:
#                     imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
#                 if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
#         imageBlank = np.zeros((height, width, 3), np.uint8)
#         hor = [imageBlank]*rows
#         hor_con = [imageBlank]*rows
#         for x in range(0, rows):
#             hor[x] = np.hstack(imgArray[x])
#         ver = np.vstack(hor)
#     else:
#         for x in range(0, rows):
#             if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
#                 imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
#             else:
#                 imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
#             if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
#         hor= np.hstack(imgArray)
#         ver = hor
#     return ver

# def getContours(img,imgContour):

#     contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#     for cnt in contours:
#         area = cv2.contourArea(cnt)
#         areaMin = cv2.getTrackbarPos("Area", "Parameters")
#         if area > areaMin:
#             cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 7)
#             peri = cv2.arcLength(cnt, True)
#             approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
#             print(len(approx))
#             x , y , w, h = cv2.boundingRect(approx)
#             cv2.rectangle(imgContour, (x , y ), (x + w , y + h ), (0, 255, 0), 5)

#             cv2.putText(imgContour, "Points: " + str(len(approx)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7,
#                         (0, 255, 0), 2)
#             cv2.putText(imgContour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7,
#                         (0, 255, 0), 2)
#             cv2.putText(imgContour, " " + str(int(x))+ " "+str(int(y)), (x - 20, y- 45), cv2.FONT_HERSHEY_COMPLEX, 0.7,
#                         (0, 255, 0), 2)

#             cx = int(x + (w / 2))
#             cy = int(y + (h / 2))

#             if (cx <int(frameWidth/2)-deadZone):
#                 cv2.putText(imgContour, " GO LEFT " , (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
#                 cv2.rectangle(imgContour,(0,int(frameHeight/2-deadZone)),(int(frameWidth/2)-deadZone,int(frameHeight/2)+deadZone),(0,0,255),cv2.FILLED)
#             elif (cx > int(frameWidth / 2) + deadZone):
#                 cv2.putText(imgContour, " GO RIGHT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
#                 cv2.rectangle(imgContour,(int(frameWidth/2+deadZone),int(frameHeight/2-deadZone)),(frameWidth,int(frameHeight/2)+deadZone),(0,0,255),cv2.FILLED)
#             elif (cy < int(frameHeight / 2) - deadZone):
#                 cv2.putText(imgContour, " GO UP ", (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
#                 cv2.rectangle(imgContour,(int(frameWidth/2-deadZone),0),(int(frameWidth/2+deadZone),int(frameHeight/2)-deadZone),(0,0,255),cv2.FILLED)
#             elif (cy > int(frameHeight / 2) + deadZone):
#                 cv2.putText(imgContour, " GO DOWN ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1,(0, 0, 255), 3)
#                 cv2.rectangle(imgContour,(int(frameWidth/2-deadZone),int(frameHeight/2)+deadZone),(int(frameWidth/2+deadZone),frameHeight),(0,0,255),cv2.FILLED)


#             cv2.line(imgContour, (int(frameWidth/2),int(frameHeight/2)), (cx,cy),
#                      (0, 0, 255), 3)

def displayPartition(img, targetX, targetY):
    # draw vertical
    line = 3
    for x in range(line):
        cv2.line(img,(int(frameWidth/line) * x,0),(int(frameWidth/line) * x,frameHeight),(255,255,0),1)
        
    for x in range(line):
        cv2.line(img, (0,int(frameHeight / line) * x), (frameWidth,int(frameHeight / line) * x), (255, 255, 0), 1)

    center_x = int(frameWidth/2)
    center_y = int(frameHeight/2)
    cv2.circle(img,(center_x,center_y),2,(0,0,255),2)
    cv2.putText(img, "({},{})".format(center_x, center_y), (center_x, center_y + 20), cv2.FONT_HERSHEY_COMPLEX, 0.5,(0, 0, 255), 1)


    if targetX > 0 and targetY > 0:
        cv2.putText(img, "({},{})".format(targetX, targetY), (targetX, targetY + 20), cv2.FONT_HERSHEY_COMPLEX, 0.5,(0, 0, 255), 1)

        if targetY < int(frameHeight/3):
            cv2.putText(img, "FORWARD", (20, 50), cv2.FONT_HERSHEY_COMPLEX,0.8,(0, 0, 255), 3)
        if targetY > int(frameHeight/3) * 2:
            cv2.putText(img, "BACKWARD", (20, 50), cv2.FONT_HERSHEY_COMPLEX,0.8,(0, 0, 255), 3)
        

        if targetX < int(frameWidth/3):
            cv2.putText(img, "LEFT", (20, 80), cv2.FONT_HERSHEY_COMPLEX,0.8,(0, 0, 255), 3)
        if targetX > int(frameWidth/3) * 2:
            cv2.putText(img, "RIGHT", (20, 80), cv2.FONT_HERSHEY_COMPLEX,0.8,(0, 0, 255), 3)




if __name__ == '__main__':

    with open('calibration.yaml', 'r') as stream:
        parameters = yaml.load(stream)
    print(parameters)

    while True:
        # Wait for the next frame
        retval, frame = cap.read()
        
        at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

        # cameraMatrix = np.array(parameters['camera_matrix']['K']).reshape((3,3))
        cameraMatrix = np.array(parameters['camera_matrix'])
        camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
        tags = at_detector.detect(gray, True, camera_params, tag_size)
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
            # tagFamily = r.tag_family.decode("utf-8")
            # cv2.putText(frame, tagFamily, (ptA[0], ptA[1] - 15),
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # print(tags)
            # print("[INFO] tag family: {}".format(tagFamily))

        pitch = 0
        roll = 0

        if len(tags) > 0:
            pitch, roll = imageUtil.getMatrix(frame, frameWidth, frameHeight, int(tags[0].center[0]), int(tags[0].center[1]))
            length = imageUtil.getHorizontalLength(tags[0].corners)
            print("length={}".format(length))

            # displayPartition(frame, int(tags[0].center[0]), int(tags[0].center[1]))
        else:
            # displayPartition(frame, 0,0)
            pitch, roll = imageUtil.getMatrix(frame, frameWidth, frameHeight, -1, -1)
        
        print("pitch={}, roll={}".format(pitch, roll))

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
