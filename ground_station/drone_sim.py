import cv2, imutils, socket
import numpy as np
import time
import base64
import threading
import errno

BUFF_SIZE = 65536
DESTINATION_IP = "127.0.0.1"
DESTINATION_PORT = 9998
IS_LISTEN = True

def listenInMessage():
    while IS_LISTEN:
        try:
            msg, client_addr = in_message_socket.recvfrom(BUFF_SIZE)
        except socket.error as e:
            err = e.args[0]
            if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                continue
            else:
                # a "real" error occurred
                print('socket error', e)
                break
        else:
            print(msg)
            #         data = base64.b64decode(packet, ' /')
            # npdata = np.fromstring(data, dtype=np.uint8)
            # frame = cv2.imdecode(npdata,1)
            # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
            # self.lblVideo.setPixmap(QtGui.QPixmap.fromImage(image))
            # key = cv2.waitKey(1) & 0xFF
            # if key == ord('q'):
            #     break
            

        
out_stream_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
in_message_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
in_message_socket.bind(("127.0.0.1", 9999))
in_message_thread = threading.Thread(target=listenInMessage, args=())
in_message_thread.start()
# out_stream_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFF_SIZE)
# host_name = socket.gethostname()
# host_ip = socket.gethostbyname(host_name)
# # host_ip = "DESTINATION_IP"
# print(host_ip)
# port = 9999
# socket_address = (host_ip, port)
# out_stream_socket.bind(socket_address)


vid = cv2.VideoCapture(0)
fps, st, frames_to_count, cnt = (0,0,20,0)

while True:
    # msg, client_addr = out_stream_socket.recvfrom(BUFF_SIZE)
    # print('GOTconnection from ', client_addr)
    WIDTH = 400
    while(vid.isOpened()):
        _, frame = vid.read()
        frame = imutils.resize(frame, width=WIDTH)
        encoded,buffer = cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,80])
        message = base64.b64encode(buffer)
        out_stream_socket.sendto(message, (DESTINATION_IP, DESTINATION_PORT))
        frame = cv2.putText(frame,'FPS: '+str(fps),(10,40),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
        cv2.imshow('TRANSMITTING VIDEO',frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            out_stream_socket.close()
            break
        if cnt == frames_to_count:
            try:
                fps = round(frames_to_count/(time.time()-st))
                st=time.time()
                cnt=0
            except:
                pass
        cnt+=1

IS_LISTEN = False
