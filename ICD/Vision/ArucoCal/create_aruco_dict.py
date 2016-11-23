import cv2
import cv2.aruco as ar
import pickle

arudict = ar.custom_dictionary(8, 4)
with open('4x4_8_dict.pickle', 'wb') as f:
    pickle.dump((arudict.bytesList, arudict.markerSize, arudict.maxCorrectionBits), f)

print(arudict)
