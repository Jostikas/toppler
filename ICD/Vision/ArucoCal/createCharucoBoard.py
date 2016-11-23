import cv2

cha_board = cv2.aruco.CharucoBoard_create(4, 6, 50, 30, dictionary=cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50))
board_image = cha_board.draw((2100, 2970), marginSize=100, borderBits=1)
cv2.imshow('Charucoboard', board_image)
cv2.waitKey(0)
cv2.imwrite('CalibBoard.png', board_image)
