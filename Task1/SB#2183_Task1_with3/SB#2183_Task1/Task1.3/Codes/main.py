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
from aruco_lib import *
import copy



########################################################################
## using os to generalise Input-Output
########################################################################
codes_folder_path = os.path.abspath('.')
images_folder_path = os.path.abspath(os.path.join('..', 'Videos'))
generated_folder_path = os.path.abspath(os.path.join('..', 'Generated'))




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

def process(ip_image):
    ###########################
    ## Your Code goes here
    ###########################
    id_list = []
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
    #detect aruco
    id1= detect_Aruco(ip_image1)
    img1=mark_Aruco(ip_image1,id1)
   
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
    cv2.imwrite("../Generated/aruco_with_id.png",ip_image)
    return ip_image, id_list


    
####################################################################
## The main program which provides read in input of one image at a
## time to process function in which you will code your generalized
## output computing code
## Do not modify this code!!!
####################################################################
def main(val):
    ################################################################
    ## variable declarations
    ################################################################
    i = 1
    ## reading in video 
    cap = cv2.VideoCapture(images_folder_path+"/"+"ArUco_bot.mp4")
    ## getting the frames per second value of input video
    fps = cap.get(cv2.CAP_PROP_FPS)
    ## getting the frame sequence
    frame_seq = int(val)*fps
    ## setting the video counter to frame sequence
    cap.set(1,frame_seq)
    ## reading in the frame
    ret, frame = cap.read()
    ## verifying frame has content
    print(frame.shape)
    ## display to see if the frame is correct
    cv2.imshow("window", frame)
    cv2.waitKey(0);
    ## calling the algorithm function
    op_image, aruco_info = process(frame)
    ## saving the output in  a list variable
    line = [str(i), "Aruco_bot.jpg" , str(aruco_info[0]), str(aruco_info[3])]
    ## incrementing counter variable
    i+=1
    ## verifying all data
    print(line)
    ## writing to angles.csv in Generated folder without spaces
    with open(generated_folder_path+"/"+'output.csv', 'w') as writeFile:
        print("About to write csv")
        writer = csv.writer(writeFile)
        writer.writerow(line)
    ## closing csv file    
    writeFile.close()



    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main(input("time value in seconds:"))
