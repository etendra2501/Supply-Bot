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
import copy






############################################
## Build your algorithm in this function
## ip_image: is the array of the input image
## imshow helps you view that you have loaded
## the corresponding image
############################################
def process(ip_image):
    ###########################
    ## Your Code goes here
    ###########################
    #Reading the image
    bool1=bool2=bool3=False 
    output_image=ip_image
    ip_image= cv2.medianBlur(ip_image, 5)
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
            main()
        else:
            y_green=center_green[1]
            x_green=center_green[0]
            y_red=center_red[1]
            x_red=center_red[0]
            #finding the angle
            angle = math.acos((((x_red-centre_x)*(x_green-centre_x))+
                               ((y_red-centre_y)*(y_green-centre_y)))/
                              (math.sqrt((centre_x-x_green)**2+
                                (y_green-centre_y)**2)*math.sqrt(((x_red-centre_x)**2+
                                (y_red-centre_y)**2))))*(180/3.14)
            angle=round(angle,2)

            #adding text to image
            org=(50, 50) #bottom_coordinates 
            font = cv2.FONT_HERSHEY_SIMPLEX#font
            fontScale = 0.5 # fontScale 
            color = (0, 0, 255) # color in BGR 
            thickness = 2	# Line thickness 
            output_image = cv2.putText(output_image, 'Angle:'+str(angle), org, font,fontScale, color, thickness, cv2.LINE_AA)

      
            return output_image

    
####################################################################
## The main program which provides read in input of one image at a
## time to process function in which you will code your generalized
## output computing code
## Modify the image name as per instruction
####################################################################
def main():
    ################################################################
    ## variable declarations
    ################################################################
    i = 1
    ## reading in video 
    cap = cv2.VideoCapture(0) #if you have a webcam on your system, then change 0 to 1
    ## getting the frames per second value of input video
    fps = cap.get(cv2.CAP_PROP_FPS)
    ## setting the video counter to frame sequence
    cap.set(3, 640)
    cap.set(4, 480)
    ## reading in the frame
    ret, frame = cap.read()
    ## verifying frame has content
    print(frame.shape)
    while(ret):
        ret, frame = cap.read()
        ## display to see if the frame is correct
        cv2.imshow("window", frame)
        cv2.waitKey(int(1000/fps));
        ## calling the algorithm function
        op_image = process(frame)
        cv2.imwrite("SB#2183_task3I.jpg",op_image)
        

    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()
