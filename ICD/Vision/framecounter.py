import cv2

vid = cv2.VideoCapture(1)
vid.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
vid.set(cv2.CAP_PROP_FPS, 125)

while True:
    ret, frame = vid.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('blah', frame)
    key = cv2.waitKey(1) & 0xff
    if key == 27:
        break
    elif key != -1:
        print('Keycode: ', key)


