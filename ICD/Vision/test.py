import cv2

logi = cv2.VideoCapture(1)
pseye = cv2.VideoCapture(2)

pseye.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
pseye.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
pseye.set(cv2.CAP_PROP_FPS, 30)
logi.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
logi.set(cv2.CAP_PROP_FRAME_HEIGHT, 448)
logi.set(cv2.CAP_PROP_FPS, 30)

cv2.namedWindow('LOGITECH', True)
cv2.namedWindow('PS3EYE', True)
cv2.moveWindow('PSEye', 850, 200)

while True:
    logi.grab()
    pseye.grab()

    ret, l_frame = logi.retrieve()
    ret, p_frame = pseye.retrieve()
    cv2.imshow('LOGITECH', l_frame)
    cv2.imshow('PS3EYE', p_frame)
    if cv2.waitKey(1) != -1:
        break
