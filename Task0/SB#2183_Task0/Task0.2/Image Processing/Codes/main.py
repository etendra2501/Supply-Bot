import cv2
import numpy as np
import os
import csv

def partA():
    img1=cv2.imread("../Images/bird.jpg")
    shape=img1.shape
    arr=[]
    arr.append(["bird.jpg",shape[0],shape[1],shape[2],img1[round((shape[0])/2)][round((shape[1])/2)][0],img1[round((shape[0])/2)][round((shape[1])/2)][1],img1[round((shape[0])/2)][round((shape[1])/2)][2]])
    
    img1=cv2.imread("../Images/cat.jpg")
    shape=img1.shape
    arr.append(["cat.jpg",shape[0],shape[1],shape[2],img1[round((shape[0])/2)][round((shape[1])/2)][0],img1[round((shape[0])/2)][round((shape[1])/2)][1],img1[round((shape[0])/2)][round((shape[1])/2)][2]])

    
    img1=cv2.imread("../Images/flowers.jpg")
    shape=img1.shape
    arr.append(["flowers.jpg",shape[0],shape[1],shape[2],img1[round((shape[0])/2)][round((shape[1])/2)][0],img1[round((shape[0])/2)][round((shape[1])/2)][1],img1[round((shape[0])/2)][round((shape[1])/2)][2]])


    img1=cv2.imread("../Images/horse.jpg")
    shape=img1.shape
    arr.append(["horse.jpg",shape[0],shape[1],shape[2],img1[round((shape[0])/2)][round((shape[1])/2)][0],img1[round((shape[0])/2)][round((shape[1])/2)][1],img1[round((shape[0])/2)][round((shape[1])/2)][2]])
    
    
    with open('../Generated/stats.csv', 'w',newline='') as f:
        writer = csv.writer(f)
        for row in arr:
            writer.writerow(row)
    f.close()
    return

def partB():
    img1=cv2.imread("../Images/cat.jpg")
    shape=img1.shape
    img1[:,:,0]=img1[:,:,1]=0
    cv2.imwrite("../Generated/cat_red.jpg",img1)
    return

def partC():
    img1=cv2.imread("../Images/flowers.jpg")
    shape=img1.shape
    alpha_channel=np.ones((shape[0],shape[1]))*127.5
    img2=np.dstack((img1,alpha_channel))
    cv2.imwrite("../Generated/flowers_alpha.png",img2)
    return

def partD():
    img1=cv2.imread("../Images/horse.jpg")
    shape=img1.shape
    arr=np.ones((shape[0],shape[1]))
    for i in range (shape[0]):
        for j in range (shape[1]):
            arr[i][j]=(0.3*img1[i][j][2])+(0.59*img1[i][j][1])+(0.11*img1[i][j][0])
    cv2.imwrite("../Generated/horse_gray.jpg",arr)
            
    return

partA()
partB()
partC()
partD()
