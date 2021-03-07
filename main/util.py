import cv2
import numpy 

class ImageUtil():

    def __init__(self):
        pass

    def getMatrix(self, frame, width, height,targetX, targetY):

        row = 11
        column = 11

        pitch = 0
        roll = 0

        for x in range(column):
            leftX = int(width/column) * x
            rightX = int(width/column) * (x+1)
            cv2.line(frame,(leftX,0),(leftX,height),(255,255,0),1)
            for y in range(row):
                topY = int(height / row) * y
                botY = int(height / row) * (y+1)
                cv2.line(frame, (0,topY), (width,topY), (255, 255, 0), 1)
                cell = [[leftX, topY], [rightX, topY],[ leftX, botY], [rightX, botY]]
                cv2.putText(frame, "({},{})".format(cell[0][0], cell[0][1]), (cell[0][0], cell[0][1] + 10), cv2.FONT_HERSHEY_COMPLEX,0.3,(0, 0, 255), 1)
                cv2.putText(frame, "({},{})".format(x, y), (cell[0][0], cell[0][1] +20), cv2.FONT_HERSHEY_COMPLEX,0.3,(0, 0, 255), 1)
                pitchSpeed = ((y - 5) * -1)/10
                rollSpeed = (x - 5)/10
                cv2.putText(frame, "({},{})".format(pitchSpeed, rollSpeed), (cell[0][0], cell[0][1] +30), cv2.FONT_HERSHEY_COMPLEX,0.3,(0, 0, 255), 1)
                if(rollSpeed == 0 and pitchSpeed == 0):
                    cv2.rectangle(frame, (cell[0][0], cell[0][1]), (cell[3][0], cell[3][1]), (0, 0, 255), -1)
                if(targetX >= leftX and targetX <= rightX):
                    if(targetY >= topY and targetY <= botY):
                        # cv2.rectangle(frame, (cell[0][0], cell[0][1]), (cell[3][0], cell[3][1]), (0, 255, 0), -1)
                        pitch = pitchSpeed
                        roll = rollSpeed
        cv2.putText(frame, "SIZE ({}{})".format(width,height), (20, 400), cv2.FONT_HERSHEY_COMPLEX,0.8,(0, 0, 255), 3)

        return pitch, roll


    def getHorizontalLength(self, corners):
        dist = numpy.linalg.norm(corners[1]-corners[0])
        return dist
        # sizePixelX = int(((corners[1][0] - corners[0][0]) + (corners[2][0] - corners[3][0]))/2)
        # sizePixelY = int(((corners[3][1] - corners[1][1]) + (corners[2][1] - corners[0][1]))/2)
        # return (sizePixelX, sizePixelY)


if __name__ == '__main__':
    image = ImageUtil()
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        height, width, channels = frame.shape

        pitch, roll =image.getMatrix(frame, width, height, 600,300)
        print("{},{}".format(pitch, roll))

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

