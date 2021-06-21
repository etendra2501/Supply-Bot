###############################################################################
## Author: Team Supply Bot
## Edition: eYRC 2019-20
## Instructions: Do Not modify the basic skeletal structure of given APIs!!!
###############################################################################


######################
## Essential libraries
######################
import cv2
import numpy as np
import os
import math
import csv
import cv2.aruco as aruco
from aruco_lib1 import *
import copy
import serial



########################################################################
## using os to generalise Input-Output
########################################################################
#codes_folder_path = os.path.abspath('.')
#images_folder_path = os.path.abspath(os.path.join('..', 'Videos'))
#generated_folder_path = os.path.abspath(os.path.join('..', 'Generated'))




############################################
## Build your algorithm in this function
## ip_image: is the array of the input image
## imshow helps you view that you have loaded
## the corresponding image
############################################
def bluredge(img, d=31):
    h, w  = img.shape[:2]
    img_pad = cv2.copyMakeBorder(img, d, d, d, d, cv2.BORDER_WRAP)
    img_blur = cv2.GaussianBlur(img_pad, (2*d+1, 2*d+1), -1)[d:-d,d:-d]
    y, x = np.indices((h, w))
    dist = np.dstack([x, w-x-1, y, h-y-1]).min(-1)
    w = np.minimum(np.float32(dist)/d, 1.0)
    return img*w + img_blur*(1-w)

def motion_kernel(angle, d, sz=65):
    kern = np.ones((1, d), np.float32)
    c, s = np.cos(angle), np.sin(angle)
    A = np.float32([[c, -s, 0], [s, c, 0]])
    sz2 = sz // 2
    A[:,2] = (sz2, sz2) - np.dot(A[:,:2], ((d-1)*0.5, 0))
    kern = cv2.warpAffine(kern, A, (sz, sz), flags=cv2.INTER_CUBIC)
    return kern

def defocus_kernel(d, sz=65):
    kern = np.zeros((sz, sz), np.uint8)
    cv2.circle(kern, (sz, sz), d, 255, -1, cv2.LINE_AA, shift=1)
    kern = np.float32(kern) / 255.0
    return kernq

def scale(X,x_min,x_max):
    nom=(X-X.min(axis=0))*(x_max-x_min)
    denom=X.max(axis=0)-X.min(axis=0)
    denom[denom==0]=1
    return x_min+nom/denom


def aruco_detection_image(ip_image):
    id_list = []
    #cv2.imshow("window_3",ip_image)
    id1= detect_Aruco(ip_image)
    #print(id1)
    if id1==None:
        aruco_detection_image(ip_image)
    if len(id1)<1:
        aruco_detection_image(ip_image)
    img1,aruco_centre=mark_Aruco(ip_image,id1)
   
    #print(id1)
    robot_state = calculate_Robot_State(img1,id1)
    #print(robot_state)
    #cv2.imshow('window3',img1)
    ip_image=img1
    #print(id1)
    #print(id1[0])
    for i in robot_state:
        id_list=robot_state[i]
    #print(id_list)
    cv2.imshow("window_1",img1)
    #cv2.imwrite("aruco_with_id.png",ip_image)
    return 



    
####################################################################
## The main program which provides read in input of one image at a
## time to process function in which you will code your generalized
## output computing code
## Do not modify this code!!!
####################################################################
def main():
    cap = cv2.VideoCapture(0) #if you have a webcam on your system, then change 0 to 1
    ## getting the frames per second value of input video
    fps = cap.get(cv2.CAP_PROP_FPS)
    ## setting the video counter to frame sequence
    cap.set(3, 640)
    cap.set(4, 480)
    ## reading in the frame
    ret,frame = cap.read()
    ## verifying frame has content
    #print(frame.shape)
    while True:
                ret, frame = cap.read()

                ## display to see if the frame is correct
                cv2.imshow("window", frame)
                cv2.waitKey(int(1000/fps))
                aruco_detection_image(frame)
                main()
                #if(cv2.waitKey(1)&0xFF==ord('q')):
                 #   break

    cap.release()
    cv2.destroyAllWindows()
                
                ## calling the algorithm function
                #ip_image,aruco_centre=aruco_detection_image(frame)
                #print("aruco detected")
               # cv2.imshow('window_2',ip_image)
                #op_image,x_green,y_green,x_red,y_red,centre_x,centre_y=coin_detection_image(frame)
                #print("coins detected")
               # cv2.imshow("window_3",op_image)
                #break
    #angle_aruco_r=math.acos((((x_red-centre_x)*(aruco_centre[0]-centre_x))+((y_red-centre_y)*(aruco_centre[1]-centre_y)))/
     #                       (math.sqrt((centre_x-aruco_centre[0])**2+(aruco_centre[1]-centre_y)**2)*math.sqrt(((x_red-centre_x)**2+(y_red-centre_y)**2))))*(180/3.14)
    #angle_aruco_g=math.acos((((x_green-centre_x)*(aruco_centre[0]-centre_x))+((y_green-centre_y)*(aruco_centre[1]-centre_y)))/
     #                       (math.sqrt((centre_x-aruco_centre[0])**2+ (aruco_centre[1]-centre_y)**2)*math.sqrt(((x_green-centre_x)**2+(y_green-centre_y)**2))))*(180/3.14)
                #cv2.imwrite("aruco_detected.jpg",ip_image)
                #cv2.imwrite("coins_detected.jpg",op_image)
                        #cv2.imwrite("aruco_detected.jpg",op_image)
                #ser=serial.Serial(port='COM5',baudrate=9600)
                #ser.close()
                #ser.open()
                #ser.write(str.encode("Hai"))
                #ser.write(chr(0x22).encode('utf-8'))
                #ser.close()
                        
           


    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    print("medical aid 6")
    print("supply bot 3 9")
    main()
 
	
