import cv2

vid = cv2.VideoCapture(0)
vid.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
vid.set(cv2.CAP_PROP_FPS, 60)

while True:
    ret, frame = vid.read()
    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imwrite('./ROIS/fullframe.png', frame)
    cv2.imshow('blah', frame)
    key = cv2.waitKey(100) & 0xff
    if key == 27:
        break
    elif key != -1:
        print('Keycode: ', key)


