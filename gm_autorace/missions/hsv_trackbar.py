import cv2
import numpy as np
import subprocess
import shlex

cap = cv2.VideoCapture(0)

def callback(x):
    global H_low,H_high,S_low,S_high,V_low,V_high
    H_low = cv2.getTrackbarPos('low H','controls')
    H_high = cv2.getTrackbarPos('high H','controls')
    S_low = cv2.getTrackbarPos('low S','controls')
    S_high = cv2.getTrackbarPos('high S','controls')
    V_low = cv2.getTrackbarPos('low V','controls')
    V_high = cv2.getTrackbarPos('high V','controls')

cv2.namedWindow('controls',2)
cv2.resizeWindow('controls', 550,200)

H_low = 0
H_high = 180
S_low = 0
S_high = 255
V_low = 0
V_high = 255

cv2.createTrackbar('low H','controls',0,180,callback)
cv2.createTrackbar('high H','controls',180,180,callback)

cv2.createTrackbar('low S','controls',0,255,callback)
cv2.createTrackbar('high S','controls',255,255,callback)

cv2.createTrackbar('low V','controls',0,255,callback)
cv2.createTrackbar('high V','controls',255,255,callback)


cmd = 'rpicam-vid --inline --nopreview -t 0 --codec mjpeg --width 640 --height 480 --framerate 30 -o - --camera 0'
process = subprocess.Popen(shlex.split(cmd), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
buffer = b""

while(1):
    buffer += process.stdout.read(4096)
    a = buffer.find(b'\xff\xd8')
    b = buffer.find(b'\xff\xd9')

    if a != -1 and b != -1:
        jpg = buffer[a:b+2]
        buffer = buffer[b+2:]

        bgr_frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

        if bgr_frame is not None:
            src = bgr_frame
            dst = src.copy()
            hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)

            hsv_low = np.array([H_low,S_low,V_low], np.uint8)
            hsv_high = np.array([H_high,S_high,V_high], np.uint8)

            mask = cv2.inRange(hsv, hsv_low, hsv_high)

            cv2.imshow('mask',mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

cv2.waitKey(0)
cv2.destroyAllWindows()
