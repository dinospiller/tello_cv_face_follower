import cv2
import numpy as np


dummy_img=np.zeros(shape=(200,100))
print ("image shape: ",dummy_img.shape[1])

while(1):
    cv2.imshow('img',dummy_img)
    k = cv2.waitKey(33)
    if k==27:    # Esc key to stop
        break
    elif k==-1:  # normally -1 returned,so don't print it
        continue
    else:
        print("you pressed: ",k) # else print its value