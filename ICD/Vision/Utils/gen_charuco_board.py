import cv2
import numpy as np
import cv2.aruco as aruco



Dictionary = aruco.getPredefinedDictionary(aruco.PREDEFINED_DICTIONARY_NAME(DICT_5X5_250=6))

aruco.drawMarker(Dictionary,5,250,markerImage,1)
aruco.drawMarker(Dictionary,10,250,markerImage,1)
aruco.drawMarker(Dictionary,20,250,markerImage,1)
aruco.drawMarker(Dictionary,25,250,markerImage,1)
aruco.drawMarker(Dictionary,50,250,markerImage,1)
aruco.drawMarker(Dictionary,100,250,markerImage,1)
aruco.drawMarker(Dictionary,200,250,markerImage,1)

cv2.imshow("markers",markerImage)
cv2.waitKey(0)

cv2.imwrite('marker.jpg',markerImage)

