import cv2 as cv
import numpy as np
import serial

global configname1, configname2
configname1 = "CONFIG1"
configname2 = "CONFIG2"
serial_port = '/dev/cu.usbmodem142201'
serial_speed = '9600'
ser = serial.Serial(serial_port, serial_speed)


def nothing(N):
    pass


def createConfig():
    cv.namedWindow(configname1)
    cv.createTrackbar("H_MIN", configname1, 0, 255, nothing)
    cv.createTrackbar("H_MAX", configname1, 255, 255, nothing)
    cv.createTrackbar("S_MIN", configname1, 0, 255, nothing)
    cv.createTrackbar("S_MAX", configname1, 255, 255, nothing)
    cv.createTrackbar("V_MIN", configname1, 0, 255, nothing)
    cv.createTrackbar("V_MAX", configname1, 255, 255, nothing)
    cv.createTrackbar("OpenSize", configname1, 0, 15, nothing)
    cv.createTrackbar("CloseSize", configname1, 0, 15, nothing)

    cv.namedWindow(configname2)
    cv.createTrackbar("H_MIN", configname2, 0, 255, nothing)
    cv.createTrackbar("H_MAX", configname2, 255, 255, nothing)
    cv.createTrackbar("S_MIN", configname2, 0, 255, nothing)
    cv.createTrackbar("S_MAX", configname2, 255, 255, nothing)
    cv.createTrackbar("V_MIN", configname2, 0, 255, nothing)
    cv.createTrackbar("V_MAX", configname2, 255, 255, nothing)
    cv.createTrackbar("OpenSize", configname2, 0, 15, nothing)
    cv.createTrackbar("CloseSize", configname2, 0, 15, nothing)

 
def labeling(frame, mask):
    labels, label, stat, centroid = cv.connectedComponentsWithStats(mask)
    maxsize = -1
    current = 0
    for i in range(1, labels):
        area = stat[i, cv.CC_STAT_AREA] 
        if maxsize < area:
            maxsize = area
            current = i

    height = stat[current, cv.CC_STAT_HEIGHT]
    left = stat[current, cv.CC_STAT_LEFT]
    top = stat[current, cv.CC_STAT_TOP]
    width = stat[current, cv.CC_STAT_WIDTH]
    center = (int(left + width / 2), int(top + height / 2))

    cv.rectangle(frame, (left, top), (left + width, top + height), (0, 0, 255), 3)
    cv.circle(frame, center, 4, (0, 255, 0), 4)
    cv.putText(frame, str(center), ((center[0]), center[1] + 40), 1, 3, (0, 255, 0), 3)
    return int(left + width / 2), int(top + height / 2)


def mouse_callback(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
        color = np.uint8([[frame[y, x]]])
        hsvc = (cv.cvtColor(color, cv.COLOR_BGR2HSV))[0, 0]
        print(hsvc)

        H = hsvc[0]
        S = hsvc[1]
        V = hsvc[2]

        if H - 30 < 0:
            cv.setTrackbarPos("H_MIN", configname1, 0)
            cv.setTrackbarPos("S_MIN", configname1, S - 30)
            cv.setTrackbarPos("V_MIN", configname1, V - 30)
            cv.setTrackbarPos("H_MAX", configname1, H + 30)
            cv.setTrackbarPos("S_MAX", configname1, S + 30)
            cv.setTrackbarPos("V_MAX", configname1, V + 30)

        else:
            cv.setTrackbarPos("H_MIN", configname1, H - 30)
            cv.setTrackbarPos("S_MIN", configname1, S - 30)
            cv.setTrackbarPos("V_MIN", configname1, V - 30)
            cv.setTrackbarPos("H_MAX", configname1, H + 30)
            cv.setTrackbarPos("S_MAX", configname1, S + 30)
            cv.setTrackbarPos("V_MAX", configname1, V + 30)

    elif event == cv.EVENT_MBUTTONDOWN:
        color = np.uint8([[frame[y, x]]])
        hsvc = (cv.cvtColor(color, cv.COLOR_BGR2HSV))[0, 0]
        print(hsvc)

        H = hsvc[0]
        S = hsvc[1]
        V = hsvc[2]

        if H - 30 < 0:
            cv.setTrackbarPos("H_MIN", configname2, 0)
            cv.setTrackbarPos("S_MIN", configname2, S - 30)
            cv.setTrackbarPos("V_MIN", configname2, V - 30)
            cv.setTrackbarPos("H_MAX", configname2, H + 30)
            cv.setTrackbarPos("S_MAX", configname2, S + 30)
            cv.setTrackbarPos("V_MAX", configname2, V + 30)

        else:
            cv.setTrackbarPos("H_MIN", configname2, H - 30)
            cv.setTrackbarPos("S_MIN", configname2, S - 30)
            cv.setTrackbarPos("V_MIN", configname2, V - 30)
            cv.setTrackbarPos("H_MAX", configname2, H + 30)
            cv.setTrackbarPos("S_MAX", configname2, S + 30)
            cv.setTrackbarPos("V_MAX", configname2, V + 30)


cam1 = cv.VideoCapture(0)
cam2 = cv.VideoCapture(1)

cam1.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cam1.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
cam2.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cam2.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

if cam1.isOpened and cam2.isOpened:
    pass
else:
    print("ERROR : Can't Open Cam")

createConfig()

cv.namedWindow("Frame")
cv.setMouseCallback("Frame", mouse_callback)

while True:
    low1 = np.array([cv.getTrackbarPos("H_MIN", configname1), cv.getTrackbarPos("S_MIN", configname1),
                     cv.getTrackbarPos("V_MIN", configname1)])
    high1 = np.array([cv.getTrackbarPos("H_MAX", configname1), cv.getTrackbarPos("S_MAX", configname1),
                      cv.getTrackbarPos("V_MAX", configname1)])
    low2 = np.array([cv.getTrackbarPos("H_MIN", configname2), cv.getTrackbarPos("S_MIN", configname2),
                     cv.getTrackbarPos("V_MIN", configname2)])
    high2 = np.array([cv.getTrackbarPos("H_MAX", configname2), cv.getTrackbarPos("S_MAX", configname2),
                      cv.getTrackbarPos("V_MAX", configname2)])
    OSize1 = cv.getTrackbarPos("OpenSize", configname1)
    CSize1 = cv.getTrackbarPos("CloseSize", configname1)
    OSize2 = cv.getTrackbarPos("OpenSize", configname2)
    CSize2 = cv.getTrackbarPos("CloseSize", configname2)

    _, frame1 = cam1.read()
    _, frame2 = cam2.read()

    hsv1 = cv.cvtColor(frame1, cv.COLOR_BGR2HSV)
    hsv2 = cv.cvtColor(frame2, cv.COLOR_BGR2HSV)

    mask1 = cv.inRange(hsv1, low1, high1)
    mask2 = cv.inRange(hsv2, low2, high2)

    mask1 = cv.morphologyEx(mask1, cv.MORPH_OPEN, np.ones((OSize1, OSize1), np.uint8))
    mask1 = cv.morphologyEx(mask1, cv.MORPH_CLOSE, np.ones((CSize1, CSize1), np.uint8))
    mask2 = cv.morphologyEx(mask2, cv.MORPH_OPEN, np.ones((OSize2, OSize2), np.uint8))
    mask2 = cv.morphologyEx(mask2, cv.MORPH_CLOSE, np.ones((CSize2, CSize2), np.uint8))

    x, y = labeling(frame1, mask1)
    z, _ = labeling(frame2, mask2)

    mask = cv.hconcat([mask1, mask2])
    frame = cv.hconcat([frame1, frame2])
    mask = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
    frame = cv.vconcat([frame, mask])

    cv.rectangle(frame, (150, 100), (500, 380), (255, 255, 0), 2)

    cv.imshow("Frame", frame)
    #cv.imshow("Mask", mask)

    data = ser.readline().decode()
    #data = list(map(int, list(data)[:-1]))
    print(x, y, z, data)

    if z == 320:
        pass
    elif x == 320:
        pass
    else:
        if x>150 and x<500:
            if y<100:
                print("박스 위쪽!!")
                #ser.write('0\r\n'.encode())
            elif y<380:
                print("박스 안쪽!!")
                ser.write('2\r\n'.encode())
            else:
                print("박스 아래!!")
                #ser.write('0\r\n'.encode())

        elif x<150:
            if y>100 and y<380:
                print("박스 왼쪽!!")
                #ser.write('0\r\n'.encode())

        elif x>500:
            if y>100 and y<380:
                print("박스 오른쪽!!")
                #ser.write('0\r\n'.encode())


    if cv.waitKey(30) == 27:
        break
