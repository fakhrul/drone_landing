import apriltag
import numpy
import cv2
import math

class FanAprilTag:

    def __init__(self, cameraParam, distortion):
        self.cameraParam  =  cameraParam
        self.distortion = distortion

        self.detector = apriltag.Detector()
        self.cameraMatrix = numpy.array([[self.cameraParam[0], 0, self.cameraParam[2]],
                [0, self.cameraParam[1], self.cameraParam[3]],
                [0, 0, 1]])
        width = 800
        height = 600
        newMtrx, roi = cv2.getOptimalNewCameraMatrix(self.cameraMatrix, self.distortion, (width, height), 1, (width, height))
        self.fixedMatrix = newMtrx
        self.centroidRepositionY = -100
        
    def toCoords(self, detection, pos, dist, cameraMatrix):
        # retrieve rotation vectors from homography
        num, rotvec, tvec, normvec = cv2.decomposeHomographyMat(detection.homography, cameraMatrix)

        # in experimentation solution 2 always yielded the correct solution
        optI = 3
        goalrotvec = rotvec[3]

        fourdmtx = numpy.matrix([[goalrotvec[0, 0], goalrotvec[0, 1], goalrotvec[0, 2], pos[0]],
                                [goalrotvec[1, 0], goalrotvec[1, 1], goalrotvec[1, 2], pos[1]],
                                [goalrotvec[2, 0], goalrotvec[2, 1], goalrotvec[2, 2], pos[2]],
                                [0., 0., 0., 1.]])
        # decompose rotation matrix into euler angles
        vals = self.decomposeSO3(rotvec[optI])
        inverted = -numpy.linalg.inv(fourdmtx) * numpy.sign(1 - abs(vals[2]))


        theta = (vals[1]*numpy.sign(1-abs(vals[2]))) - (math.atan2((pos[0]), pos[2]))
        subtract =  -vals[0]
        if numpy.sign(1-abs(vals[2])) > 0:
            subtract = -(vals[0] + math.pi)

        phi = subtract - (math.atan2((pos[1]), pos[2]))

        # calculate planar coordinates as a function of the distance and the y rotation
        x = abs(dist * math.sin(theta))
        y = dist * math.cos(theta)
        z = dist * math.sin(phi)

        return inverted[0], inverted[2], inverted[1], theta, (vals[1]*numpy.sign(1-vals[2])), subtract

    def isLeft(self, pos, corners):
        """
        Determines if the robot is to the left or to the right of the apriltag
        from the perspective of facing the apriltag
        :param pos: translation vector of the robot (generated from homography)
        :param corners: position of the corners of the apriltag in image space
        :return: -1 if the robot is to the left, 1 if the robot is to the right
        """
        print(pos)
        print(corners)

        d1 = numpy.array([pos[0] - cornerDist, pos[1] + cornerDist, pos[2]])
        d2 = numpy.array([pos[0] + cornerDist, pos[1] + cornerDist, pos[2]])
        d3 = numpy.array([pos[0] + cornerDist, pos[1] - cornerDist, pos[2]])
        d4 = numpy.array([pos[0] - cornerDist, pos[1] - cornerDist, pos[2]])

        testRatio = (numpy.linalg.norm(d1)+numpy.linalg.norm((d4)))/(numpy.linalg.norm(d2)+numpy.linalg.norm((d3)))

        print(corners[3][1])
        print(corners[0][1])
        print(corners[2][1])
        print(corners[1][1])
        measuredRatio = (corners[3][1] - corners[0][1])/(corners[2][1]-corners[1][1])

        if testRatio > measuredRatio:
            return 1
        return -1
        
    def homographyDist(self, detection, mtx):
        """
        Computes the position of the camera, and the distance to the camera
        :param detection: apriltag detection object
        :param mtx: intrinsic camera matrix
        :return: the distance to the apriltag, and the position
        """

        # create extentrics matrix without accounting for s
        scaledExtrinsics = numpy.zeros((3, 3))
        for x in range(0, 2):
            scaledExtrinsics[x] = (detection.homography[x]-detection.homography[2]*mtx[x][2])/mtx[x][x]
        scaledExtrinsics[2] = detection.homography[2]

        # calculate s as the geometric mean of the magnitudes of the first two columns
        magnitudes = numpy.linalg.norm(scaledExtrinsics, axis=0)
        s = numpy.sqrt(magnitudes[0]+magnitudes[1])

        # ensure z is positive
        if detection.homography[2][2] < 0:
            s = s *-1

        scaledExtrinsics = scaledExtrinsics/s

        exp_factor = 1

        return scaledExtrinsics[2][2] * exp_factor, exp_factor*scaledExtrinsics[:3,2], scaledExtrinsics[1][2]*exp_factor

    def decomposeSO3(self, rotationMatrix):
        thetaX = math.atan2(rotationMatrix[2, 1], rotationMatrix[2, 2])
        thetaY = math.atan2(-rotationMatrix[2, 0], math.hypot(rotationMatrix[2, 1], rotationMatrix[2, 2]))
        thetaZ = math.atan2(rotationMatrix[1, 0], rotationMatrix[0, 0])
        return numpy.array((thetaX, thetaY, thetaZ))

    def getTagsById(self, image, tagId, cameraParam, tagSize):
        gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY) # convert to grayscale
        detections, dimg = self.detector.detect(gray, return_image=True) 
        totalDetections = len(detections)

        overlay = image // 2 + dimg[:, :, None] // 2

        foundDetection = None
        isFound = False
        foundColor = (0,255,0)
        otherColor = (0,0,255)
        pixelDiffX = 0
        pixelDiffY = 0
        degree = ()
        sizePixel = None
        for i, detection in enumerate(detections):
            # print("detected {}, requested {}".format(detection.tag_id, tagId))
            if(detection.tag_id == tagId):
                isFound = True
                foundDetection = detection

                pose, e0, e1 = self.detection_pose(detection,
                                    cameraParam,
                                    tagSize)


                # print(detection.corners.astype(int))
                # self.draw_axis(overlay, cameraParam, pose)
                # self.draw_pose(overlay,
                #                     cameraParam ,
                #                     tagSize,
                #                     pose, detection.center)
                pts = numpy.array(detection.corners, numpy.int32)
                pts = pts.reshape((-1,1,2))
                cv2.polylines(overlay,[pts],True,foundColor,1)
                # Print the tag ID
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(overlay,str(detection.tag_id), tuple(numpy.array(detection.center, numpy.int32)), font, 3,foundColor,1,cv2.LINE_AA)

                                # draw lines
                # self.draw_cross_line(overlay, int(len(overlay[0]) / 2), int(len(overlay) / 2), (255, 0, 0))
                cX = int(len(overlay[0]) / 2)
                cY = int(len(overlay) / 2)
                cv2.circle(overlay, (cX, cY), 10,(255, 0, 0), 2)
                cv2.putText(overlay, "(Target)", (cX - 30, cY + 30),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                self.draw_cross_line(overlay, int(detection.center[0]), int(detection.center[1] + self.centroidRepositionY), (0, 255, 255))

                textLine = 20
                lineToAdd = 20
                cv2.putText(overlay, "Drone center position x: {} y: {}".format(int(len(overlay[0]) / 2), int(len(overlay) / 2)), (5,textLine),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                textLine += lineToAdd
                cv2.putText(overlay, "Apriltag position x: {} y: {}".format(int(detection.center[0]), int(detection.center[1] + self.centroidRepositionY)), (5,textLine),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                textLine += lineToAdd
                pixelDiffX = int(len(overlay[0]) / 2) - int(detection.center[0])
                pixelDiffY = int(len(overlay) / 2) - int(detection.center[1] + self.centroidRepositionY)
                cv2.putText(overlay, "Difference distance between Drone and Apriltag (pixel) x: {} y: {}".format(pixelDiffX,  pixelDiffY), (5,textLine),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                textLine += lineToAdd


                # pose, e0, e1 = self.detection_pose(detection,
                #                                 self.cameraParam,
                #                                 self.tagSize)

                # print(pose)

                # distanceMatrix = pose[:3,3] 
                # distanceMsg = 'Distance (cm) x={0:.2f}, y={1:.2f}, z={0:.2f}'.format(distanceMatrix[0]*100, distanceMatrix[1]*100,distanceMatrix[2]*100)
                # cv2.putText(overlay, distanceMsg, (5,textLine),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                # textLine += lineToAdd


                rotationMatrix3d = pose[0:3,:-1]
                euler = self.rotationMatrixToEulerAngles(rotationMatrix3d)
                degree = numpy.degrees(euler) 
                # degreeMsg = 'Rotation (degree) yaw={0:.2f}, pitch={1:.2f}, roll={2:.2f}'.format(degree[0], degree[1],degree[2])
                # cv2.putText(overlay, degreeMsg, (5,textLine),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                # textLine += lineToAdd
                degreeMsg = 'Angle to apriltag (degree) {0:.2f}'.format( degree[1])
                cv2.putText(overlay, degreeMsg, (5,textLine),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                textLine += lineToAdd


                #method 2
                # dist, pose2, zPos = self.homographyDist(detection, self.fixedMatrix)
                # origin = numpy.zeros(3)
                # x, y, z, theta, phi, rho = self.toCoords(detection, pose2, dist, self.fixedMatrix)
                # if rho < 0:
                #     rho = rho+math.pi*2
                # calculatedTheta = (theta-math.pi/2)
                # # reproject onto image overlay
                # projPoints, jacobian = cv2.projectPoints(numpy.array([pose2]), origin, origin, self.fixedMatrix, self.distortion)
                # cv2.circle(overlay, (int(projPoints[0][0][0]), int(projPoints[0][0][1])), 5, (0, 0, 255))

                # cv2.putText(overlay, "Method 2  x={}, y={}, z={}".format(x, y,z ), (5,textLine), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 255),1)
                # textLine += lineToAdd

                # cv2.putText(overlay, "Method 2  theta={}, phi={}, rho={}".format(theta, phi,rho ), (5,textLine), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 255),1)
                # textLine += lineToAdd

                # cv2.putText(overlay, "Distance: {}".format(dist), (5,textLine), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 255),1)
                # textLine += lineToAdd

                sizePixelX = int(((detection.corners[1][0] - detection.corners[0][0]) + (detection.corners[2][0] - detection.corners[3][0]))/2)
                sizePixelY = int(((detection.corners[3][1] - detection.corners[1][1]) + (detection.corners[2][1] - detection.corners[0][1]))/2)
                sizePixel = (sizePixelX, sizePixelY)
                cv2.putText(overlay, "Apriltag size: {}".format(sizePixel), (5,textLine), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0, 255, 0),1)
                textLine += lineToAdd
                # sizePixel = detection.corners
                # cv2.putText(overlay, "Size: {}".format(sizePixel), (5,textLine), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 255),1)
                # textLine += lineToAdd


            else:
                pts = numpy.array(detection.corners, numpy.int32)
                pts = pts.reshape((-1,1,2))
                overlay = cv2.polylines(overlay,[pts],True,otherColor,1)
                # Print the tag ID
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(overlay,str(detection.tag_id), tuple(numpy.array(detection.center, numpy.int32)), font, 3,otherColor,1,cv2.LINE_AA)

        diffPixel = (pixelDiffX, pixelDiffY)
        return (isFound, foundDetection, overlay, diffPixel, degree, sizePixel)

    def getTags(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY) # convert to grayscale
        return self.detector.detect(gray, return_image=True) 

    def detection_pose(self, detection, camera_params, tag_size):
        return self.detector.detection_pose(detection,
                                    camera_params,
                                    tag_size)
    def draw_cross_line(self, dimg, cx, cy, color):
        cv2.line(dimg, (cx - 200, cy), (cx + 200, cy), color)
        cv2.line(dimg, (cx, cy - 200), (cx, cy + 200), color)


    def draw_axis(self, img, camera_params, pose):
        # unit is mm
        rotV, _ = cv2.Rodrigues(pose[:3,:3])
        fx, fy, cx, cy = camera_params
        K = numpy.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)
        rvec, _ = cv2.Rodrigues(pose[:3,:3])
        tvec = pose[:3, 3]
        dcoeffs = numpy.zeros(5)

        points = numpy.float32([[5, 0, 0], [0, 5, 0], [0, 0, 5], [0, 0, 0]]).reshape(-1, 3)
        axisPoints, _ = cv2.projectPoints(points, rvec, tvec, K, dcoeffs)
        ipoints = numpy.round(axisPoints).astype(int)

        img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[0].ravel()), (255,0,0), 3)
        img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[1].ravel()), (0,255,0), 3)
        img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[2].ravel()), (0,0,255), 3)
        return img

    def draw_pose(self, overlay, camera_params, tag_size, pose, center, z_sign=1):
        opoints = numpy.array([
            -1, -1, 0,
            1, -1, 0,
            1,  1, 0,
            -1,  1, 0,
            -1, -1, -2*z_sign,
            1, -1, -2*z_sign,
            1,  1, -2*z_sign,
            -1,  1, -2*z_sign,
        ]).reshape(-1, 1, 3) * 0.5*tag_size
        # opoints = numpy.array([
        #     -1, -1, 0,
        #     1, -1, 0,
        #     1,  1, 0,
        #     -1,  1, 0,
        #     -1, -1, -2*z_sign,
        #     1, -1, -2*z_sign,
        #     1,  1, -2*z_sign,
        #     -1,  1, -2*z_sign,
        # ]).reshape(-1, 1, 3) 

        edges = numpy.array([
            0, 1,
            1, 2,
            2, 3,
            3, 0,
            0, 4,
            1, 5,
            2, 6,
            3, 7,
            4, 5,
            5, 6,
            6, 7,
            7, 4
        ]).reshape(-1, 2)
            
        fx, fy, cx, cy = camera_params

        K = numpy.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

        rvec, _ = cv2.Rodrigues(pose[:3,:3])
        tvec = pose[:3, 3]

        dcoeffs = numpy.zeros(5)

        ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

        ipoints = numpy.round(ipoints).astype(int)

        ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

        # print(outerCentroid)
        for i, j in edges:
            cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)

        # #draw line center
        # outerCentroid = tuple(numpy.round(get_centroid(ipoints[4:8])).astype(int))
        # cv2.circle(overlay, outerCentroid, 1, (0, 255, 255), -1)
        innerCentroid = tuple(numpy.round(center).astype(int))
        cv2.circle(overlay, innerCentroid, 5, (0, 255, 0), -1)
        cv2.putText(overlay, "({},{})".format(innerCentroid[0],innerCentroid[1]), (innerCentroid[0] + 10,innerCentroid[1] + 5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)


        # cv2.line(overlay, outerCentroid, innerCentroid, (0, 255, 0))

        # cv2.line(overlay, (outerCentroid[0] , 0), (outerCentroid[0], len(overlay)), (255, 255, 0))
        # cv2.line(overlay, (0, outerCentroid[1]), (len(overlay[0]), outerCentroid[1]), (255, 255, 0))
        # cv2.line(overlay, (outerCentroid[0] , outerCentroid[1]), (screenCX, screenCY), (0, 0, 255))


        # cX, cY = numpy.array(center, numpy.int32)
        # cv2.line(overlay, (cX , 0), (cX, len(overlay)), (255, 255, 0))
        # cv2.line(overlay, (0, cY), (len(overlay[0]), cY), (255, 255, 0))
        # cv2.line(overlay, (cX , cY), (screenCX, screenCY), (0, 0, 255))


        # horGapPxl = abs(cX - screenCX)
        # horGapPxlMessage = 'Horizontal Gap {} pixel'.format(horGapPxl)
        # cv2.putText(overlay, horGapPxlMessage, (25,50),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # verGapPxl = abs(cY- screenCY)
        # verGapPxlMessage = 'Vertical Gap {} pixel'.format(verGapPxl)
        # cv2.putText(overlay, verGapPxlMessage, (25,75),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self,R) :
        Rt = numpy.transpose(R)
        shouldBeIdentity = numpy.dot(Rt, R)
        I = numpy.identity(3, dtype = R.dtype)
        n = numpy.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6
    
    
    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotationMatrixToEulerAngles(self,R) :
    
        assert(self.isRotationMatrix(R))
        
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        
        singular = sy < 1e-6
    
        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
    
        return numpy.array([x, y, z])

if __name__ == "__main__":
    cameraParam = (810.2467309141659, 807.5285604954187, 398.95749549897056, 317.7245812466577) # for MI camera
    distortion = numpy.zeros(5)

    vcap = cv2.VideoCapture(0)
    # vcap = cv2.VideoCapture(0)
    # vcap = cv2.VideoCapture('video/v1.mp4')
    vcap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    aprilTag = FanAprilTag(cameraParam, distortion)
    tag_id_to_process = 2
    tagSize = 0.56

    while(True):
        ret, frame = vcap.read()
        isFound, detection, dimg, pixel, degree, sizePixel = aprilTag.getTagsById(frame, tag_id_to_process, cameraParam,tagSize)

        # totalDetections = len(detections)
        # print("total detection {}".format(totalDetections))
        cv2.imshow("Tags", dimg)
        if cv2.waitKey(22) & 0xFF == ord('q'):
            break

    vcap.release()
    cv2.destroyAllWindows()

