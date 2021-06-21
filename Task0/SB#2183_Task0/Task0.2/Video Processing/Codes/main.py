import cv2
import numpy as np
import os

def partA():
    cap=cv2.VideoCapture("../Videos/RoseBloom.mp4")
    i=0
    while cap.isOpened():
        ret,frame=cap.read()
        if ret==False:
            break
        if i==125:
            cv2.imwrite("../Generated/frame_as_6.jpg",frame)
        i+=1
    cap.release()
    cv2.destroyAllWindows()
    return

def partB():
    img1=cv2.imread("../Generated/frame_as_6.jpg")
    img1[:,:,0]=img1[:,:,1]=0
    cv2.imwrite("../Generated/frame_as_6_red.jpg",img1)
    return

partA()
partB()
