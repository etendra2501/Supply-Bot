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




########################################################################
## using os to generalise Input-Output
########################################################################
codes_folder_path = os.path.abspath('.')
images_folder_path = os.path.abspath(os.path.join('..', 'Images'))
generated_folder_path = os.path.abspath(os.path.join('..', 'Generated'))




############################################
## Build your algorithm in this function
## ip_image: is the array of the input image
## imshow helps you view that you have loaded
## the corresponding image
############################################
def process(ip_image):
    ###########################
    ## Your Code goes here
    #Convert the image from BGR to HSV
    ip_image1=cv2.cvtColor(ip_image,cv2.COLOR_BGR2HSV)
    #Extracting the red circle
    lower_red = np.array([0,255,70])
    upper_red = np.array([10,255,255])
    mask1 = cv2.inRange(ip_image1, lower_red, upper_red)
    lower_red = np.array([170,255,100])
    upper_red = np.array([180,255,255])
    mask2 = cv2.inRange(ip_image1,lower_red,upper_red)
    mask = mask1+mask2
    mask1 = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
    mask1 = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((3,3),np.uint8))
    #Creating a completely white image of the same size as the original image and masking the red circle on it
    white_image=np.ones((ip_image.shape[0],ip_image.shape[1],1))*255
    mask2 = cv2.bitwise_not(mask1)
    #finding the extreme points of the circle
    for x in range(mask1.shape[0]):
        for y in range(mask1.shape[1]):
            if(mask2[x,y]==0):
                x1,y1=x,y
                break;
    for x in range(mask1.shape[0]-1,0,-1):
        for y in range(mask1.shape[1]-1,0,-1):
            if(mask2[x,y]==0):
                x2,y2=x,y
                break;
    #finding the coordinates of the centre of the red
    x_red=(x1+x2)//2
    y_red=(y1+y2)//2
    red = [0,255]
    mask2[x_red][y_red]=255
    #Extracting the green circle
    lower_green = np.array([50,255,100])
    upper_green = np.array([80,255,255])
    mask2 = cv2.inRange(ip_image1,lower_green,upper_green)
    mask1 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
    mask1 = cv2.morphologyEx(mask2, cv2.MORPH_DILATE, np.ones((3,3),np.uint8))
    #Creating a completely white image of the same size as the original image and masking the green circle on it
    white_image=np.ones((ip_image.shape[0],ip_image.shape[1],1))*255
    mask2 = cv2.bitwise_not(mask1)
    #finding the extreme points of the circle
    for x in range(mask1.shape[0]):
        for y in range(mask1.shape[1]):
            if(mask2[x,y]==0):
                x1,y1=x,y
                break;
    for x in range(mask1.shape[0]-1,0,-1):
        for y in range(mask1.shape[1]-1,0,-1):
            if(mask2[x,y]==0):
                x2,y2=x,y
                break;
    #finding the coordinates of the centre of the green circle 
    x_green=(x1+x2)//2
    y_green=(y1+y2)//2
    red = [0,255]
    mask2[x_green][y_green]=255
    #finding the coordinates of the center of the image
    centre_x,centre_y=ip_image.shape[0]//2,ip_image.shape[1]//2
    #finding the slope of the line joining the center of the circles with the center of the image 
    m1=(y_green-centre_y)/(x_green-centre_x)
    m2=(y_red-centre_y)/(x_red-centre_x)
    #finding the angle
    angle = math.acos((((x_red-centre_x)*(x_green-centre_x))+((y_red-centre_y)*(y_green-centre_y)))/(math.sqrt((centre_x-x_green)**2+(y_green-centre_y)**2)*math.sqrt(((x_red-centre_x)**2+(y_red-centre_y)**2))))*(180/3.14)
    angle=round(angle,2)
    ## Your Code goes here
    ###########################
    cv2.imshow("window",ip_image)
    cv2.waitKey(0);
    return angle




    
####################################################################
## The main program which provides read in input of one image at a
## time to process function in which you will code your generalized
## output computing code
## Do not modify this code!!!
####################################################################
def main():
    ################################################################
    ## variable declarations
    ################################################################
    i = 1
    line = []
    ## Reading 1 image at a time from the Images folder
    for image_name in os.listdir(images_folder_path):
        ## verifying name of image
        print(image_name)
        ## reading in image 
        ip_image = cv2.imread(images_folder_path+"/"+image_name)
        ## verifying image has content
        print(ip_image.shape)
        ## passing read in image to process function
        A = process(ip_image)
        ## saving the output in  a list variable
        line.append([str(i), image_name , str(A)])
        ## incrementing counter variable
        i+=1
    ## verifying all data
    print(line)
    ## writing to angles.csv in Generated folder without spaces
    with open(generated_folder_path+"/"+'angles.csv', 'w', newline='') as writeFile:
        writer = csv.writer(writeFile)
        writer.writerows(line)
    ## closing csv file    
    writeFile.close()



    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()
