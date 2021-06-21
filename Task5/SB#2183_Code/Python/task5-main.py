###############################################################################
## Author: Team Supply Bot
## Edition: eYRC 2019-20
## Instructions: Do Not modify the basic skeletal structure of given APIs!!!
###############################################################################
"""
* Team Id : SB#2183
 * Author List: Natasha Choudhary,Pavit Kaur,Ritika Gupta,Etendra Verma
 * Filename: task5-main.py
 * Theme: Supply bot
* Functions: aruco_detection_image(ip_image),coin_detection_image(ip_image),main()
* Global Variables: none 
"""

######################
## Essential libraries
######################
import cv2
import numpy as np
import os
import math
import csv
import cv2.aruco as aruco
from aruco_lib import *
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
    return kern

def scale(X,x_min,x_max):
    nom=(X-X.min(axis=0))*(x_max-x_min)
    denom=X.max(axis=0)-X.min(axis=0)
    denom[denom==0]=1
    return x_min+nom/denom

"""
 * Function Name:aruco_detection_image 
* Input: image
* Output: aruco center
* Logic: detects the aruco
* Example Call: aruco_detection_image(ip_image)
"""
def aruco_detection_image(ip_image):
    '''
    image=ip_image
    #create a blank image of same size as original image
    new_image = np.zeros(image.shape, image.dtype)
    
    alpha = 2.0 # contrast factor
    beta = 0    # brightness factor
    #adjust the contrast
    new_image = np.clip(alpha*image + beta, 0, 255)
                
    img=new_image
    #split the image
    imgB=img[:,:,0]
    imgG=img[:,:,1]
    imgR=img[:,:,2]
    #apply filters
    imgR = np.float32(imgR)/255.0
    imgG = np.float32(imgG)/255.0
    imgB = np.float32(imgB)/255.0

    imgR = bluredge(imgR)
    imgG = bluredge(imgG)
    imgB = bluredge(imgB)
    
    IMGR = cv2.dft(imgR, flags=cv2.DFT_COMPLEX_OUTPUT)
    IMGG = cv2.dft(imgG, flags=cv2.DFT_COMPLEX_OUTPUT)
    IMGB = cv2.dft(imgB, flags=cv2.DFT_COMPLEX_OUTPUT)
    def updateRed(_):
        ang = np.deg2rad(90)
        d = 20
        noise = 10**(-0.1*25)
        psf = motion_kernel(ang, d)
        psf /= psf.sum()
        psf_pad = np.zeros_like(imgR)
        kh, kw = psf.shape
        psf_pad[:kh, :kw] = psf
        PSF = cv2.dft(psf_pad, flags=cv2.DFT_COMPLEX_OUTPUT, nonzeroRows = kh)
        PSF2 = (PSF**2).sum(-1)
        iPSF = PSF / (PSF2 + noise)[...,np.newaxis]
        RES = cv2.mulSpectrums(IMGR, iPSF, 0)
        res = cv2.idft(RES, flags=cv2.DFT_SCALE | cv2.DFT_REAL_OUTPUT )
        res = np.roll(res, -kh//2, 0)
        resR = np.roll(res, -kw//2, 1)
        return resR

    def updateGreen(_):
        ang = np.deg2rad(90)
        d = 20
        noise = 10**(-0.1*25)
        psf = motion_kernel(ang, d)
        psf /= psf.sum()
        psf_pad = np.zeros_like(imgG)
        kh, kw = psf.shape
        psf_pad[:kh, :kw] = psf
        PSF = cv2.dft(psf_pad, flags=cv2.DFT_COMPLEX_OUTPUT, nonzeroRows = kh)
        PSF2 = (PSF**2).sum(-1)
        iPSF = PSF / (PSF2 + noise)[...,np.newaxis]
        RES = cv2.mulSpectrums(IMGG, iPSF, 0)
        res = cv2.idft(RES, flags=cv2.DFT_SCALE | cv2.DFT_REAL_OUTPUT )
        res = np.roll(res, -kh//2, 0)
        resG = np.roll(res, -kw//2, 1)
        return resG

    def updateBlue(_):
        ang = np.deg2rad(90)
        d = 20
        noise = 10**(-0.1*25)
        psf = motion_kernel(ang, d)
        psf /= psf.sum()
        psf_pad = np.zeros_like(imgB)
        kh, kw = psf.shape
        psf_pad[:kh, :kw] = psf
        PSF = cv2.dft(psf_pad, flags=cv2.DFT_COMPLEX_OUTPUT, nonzeroRows = kh)
        PSF2 = (PSF**2).sum(-1)
        iPSF = PSF / (PSF2 + noise)[...,np.newaxis]
        RES = cv2.mulSpectrums(IMGB, iPSF, 0)
        res = cv2.idft(RES, flags=cv2.DFT_SCALE | cv2.DFT_REAL_OUTPUT )
        res = np.roll(res, -kh//2, 0)
        resB = np.roll(res, -kw//2, 1)
        return resB
    resR=updateRed(None)
    resG=updateGreen(None)
    resB=updateBlue(None)

    ip_image1=np.dstack((resB,resG,resR))
    ip_image1=(ip_image1*255).astype('uint8')
    '''
    id_list = []
    #cv2.imshow("window_3",ip_image)
    id1= detect_Aruco(ip_image)
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
    #cv2.imshow("window_1",img1)
    #cv2.imwrite("aruco_with_id.png",ip_image)
    return ip_image,aruco_centre
def serial_stop(a):
    ser=serial.Serial(port='COM3',baudrate=9600)
    ser.close()
    ser.open()
    #ser.write(str.encode(0x38))
    ser.write(chr(a).encode('utf-8'))
    ser.close()
   
"""
 * Function Name:coin_detection_image 
* Input: image
* Output: output image
* Logic: detects the coin
* Example Call: coin_detection_image(ip_image)
"""
def coin_detection_image(ip_image):
    ip_image,aruco_centre=aruco_detection_image(ip_image)
    
    #aroko_im=ip_image
    #cv2.imshow("window_ll1",aroko_im)
    
    ###########################
    ## Your Code goes here
    ###########################
    
    bool1=bool2=bool3=False 
    output_image=ip_image
   # ip_image= cv2.medianBlur(ip_image, 5)
    #Denoising
    while (not bool1 or not bool2 or not bool3):
        ip_image=cv2.fastNlMeansDenoisingColored(ip_image,None,20,10,7,21)
        #converting image from BGR to HSV
        hsv_img=cv2.cvtColor(ip_image,cv2.COLOR_BGR2HSV)
        
        #Extracting red part of image
        lower_red=np.array([161,155,84])
        upper_red=np.array([179,255,255])
        red_mask=cv2.inRange(hsv_img,lower_red,upper_red)
        #Removing noise from mask
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_DILATE, np.ones((3,3),np.uint8))
        mask_r = cv2.bitwise_not(red_mask)
        #Finding contours in mask_r and if it is a red circle then encircle it with blue color
        contours_r,heirarchy = cv2.findContours(mask_r, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for i in contours_r:
            approx = cv2.approxPolyDP(i,0.01*cv2.arcLength(i,True),True)
            x = len(approx)
            if x>9 and x<13:
                ((x_red, y_red), radius) = cv2.minEnclosingCircle(i)#find the minimum enclosing circle for the circular contours
            
                if(radius>4 and radius<8):
                    
                    bool1=True
                    M = cv2.moments(i)
                    center_red = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    cv2.drawContours(output_image,[i],0,(255,0,0),2)

        #cv2.imshow("window_ll2",aroko_im)

    #******************
        lower_white=np.array([0,0,230])
        upper_white=np.array([255,255,255])
        white_mask=cv2.inRange(hsv_img,lower_white,upper_white)
        #Removing noise from mask
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_DILATE, np.ones((3,3),np.uint8))
        mask_w = cv2.bitwise_not(white_mask)
        #Finding contours in mask_w 
        contours_w,heirarchy = cv2.findContours(mask_w, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
         
        for i in contours_w:
            approx = cv2.approxPolyDP(i,0.01*cv2.arcLength(i,True),True)
            x = len(approx)
            if x>9 and x<13:
                ((x_red, y_red), radius) = cv2.minEnclosingCircle(i)#find the minimum enclosing circle for the circular contours
                
                if(radius>6 and radius<7.5):
                    
                    bool2=True
                    M = cv2.moments(i)
                    centre_x=int(M["m10"] / M["m00"])
                    centre_y=int(M["m01"] / M["m00"])
                    

        #Extracting green part of image
        lower_green=np.array([25,50,93])
        upper_green=np.array([60,255,255])
        green_mask=cv2.inRange(hsv_img,lower_green,upper_green)
        #Removing noise from mask
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_DILATE, np.ones((3,3),np.uint8))
        mask_g = cv2.bitwise_not(green_mask)

        #cv2.imshow("window_ll3",aroko_im)
        
        #Finding contours in mask_g and if it is a green circle then encircle it with blue color
        contours_g,heirarchy = cv2.findContours(mask_g, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for i in contours_g:
            approx = cv2.approxPolyDP(i,0.01*cv2.arcLength(i,True),True)
            x = len(approx)
            if x>9 and x<12:
                ((x_red, y_red), radius) = cv2.minEnclosingCircle(i)#find the minimum enclosing circle for the circular contours
                
                if(radius>5 and radius<8):
                    
                    bool3=True
                    M = cv2.moments(i)
                    center_green = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    cv2.drawContours(output_image,[i],0,(255,0,0),2)
        
        if (not bool1 or not bool2 or not bool3):
            #print("back 2 main")
            main()
        else:
            
            y_green=center_green[1]
            x_green=center_green[0]
            y_red=center_red[1]
            x_red=center_red[0]
            #finding the angle
            #print("medical aid 6")
            #print("food supply 3 9")
            angle=round(angle,2)
            #cv2.imshow("window_ll1",output_image)
            
            x_aruco= aruco_centre[0]
            y_aruco= aruco_centre[1]
            angle_aruco_redcoin = math.acos((((x_red-centre_x)*(x_aruco-centre_x))+
                               ((y_red-centre_y)*(y_aruco-centre_y)))/
                              (math.sqrt((centre_x-x_aruco)**2+
                                (y_aruco-centre_y)**2)*math.sqrt(((x_red-centre_x)**2+
                                (y_red-centre_y)**2))))*(180/3.14)
            if angle_aruco_redcoin in range (0,10) or angle_aruco_redcoin in range (350,360):
                serial_stop(0x38)
            else:
                main()
                
            
            
            #adding text to image
            org=(50, 50) #bottom_coordinates 
            font = cv2.FONT_HERSHEY_SIMPLEX#font
            fontScale = 0.5 # fontScale 
            color = (0, 0, 255) # color in BGR 
            thickness = 2	# Line thickness 
            output_image = cv2.putText(output_image, 'Angle:'+str(angle), org, font,fontScale, color, thickness, cv2.LINE_AA)
            #cv2.imshow("window_2",output_image)
            cv2.imwrite("image_with_coins.png",output_image)
            return output_image


    
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
    while(ret):
                ret, frame = cap.read()
                ## display to see if the frame is correct
            #    cv2.imshow("window", frame)
                cv2.waitKey(int(1000/fps))
                coin_detection_image(frame)
                
                return
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
	main()
 
	
