#!/usr/bin/env python
# -*- coding: utf-8 -*-
####################################################################
# 프로그램명 : hough_drive_c1.py
# 작 성 자 : (주)자이트론
# 생 성 일 : 2020년 07월 23일
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import rospy, rospkg, time
import numpy as np
import cv2, math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from laptop.msg import haar_detect
from math import *
import signal
import sys
import os

image = np.empty(shape=[0])
bridge = CvBridge()
cascade = cv2.CascadeClassifier('C:\\Users\\IdeaPad5\\Desktop\\cascades\\obs_cascade.xml')
Width = 320
Height = 240

numbering = 0

def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def ROI(img, vertices, color3=(255,255,255), color1=255):
    mask = np.zeros_like(img)
    #img와 같은 크기의 배열 생성 -> 값은 0
    if len(img.shape) > 2:
        color = color3
        #채널 수가 2보다 크면 흰색(RGB)
    else:
        color = color1
        #채널 수가 2보다 작거나 같으면 흰색(RGB가 아닌 다른 방식)
        #->결과적으로 흰색을 받아냄
    cv2.fillPoly(mask, vertices, color)
    #색이 채워진 다각형 그리기
    #'mask'라는 이미지에, vertices라는 좌표점들, 색상(흰색)의 다각형을 그림
    ROI_image = cv2.bitwise_and(img, mask)
    #img와 mask의 같은 위치의 색이 흰색이면 흰색, 아니라면 검정색
    return ROI_image


def start():
    global image
    global Width, Height

    global numbering

    rospy.init_node('haar_detect')
    detected = rospy.Publisher('haar_detect', haar_detect, queue_size = 1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    t_check = time.time()
    f_n = 0
    while not rospy.is_shutdown():
        while not image.size == (Width*Height*3):
            continue
        '''
        f_n += 1
        if (time.time() - t_check) > 1:
            print("haar_fps : ", f_n)
            t_check = time.time()
            f_n = 0
        '''
        #copy_img = image.copy()


        #vertices = np.array([[(60,60),(60,180),(260,180),(260,60)]])

        #copy_img = ROI(copy_img,vertices)

        #test_roi = image[40: 220, 45 : 320]
        #resize_img = cv2.resize(test_roi, (Width, Height), interpolation = cv2.INTER_LINEAR)

        #gray = cv2.cvtColor(resize_img,cv2.COLOR_BGR2GRAY)
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        cascades = cascade.detectMultiScale(gray,1.01,50)

        #stop_cascades = stop_cascade.detectMultiScale(gray,1.01,200)

        haar_msg = haar_detect()

        if len(cascades) > 0:
            haar_msg.data = cascades[0]
            detected.publish(haar_msg)
            print("obstacle detected!!")
            sys.exit(0)
        else:
            haar_msg.data = (0,0,0,0)
            detected.publish(haar_msg)

        #cv2.imwrite('C:\\Users\\IdeaPad5\\Desktop\\negative_image_more\\negative'+str(numbering).zfill(4)+'.jpg',gray)
        #print(str(numbering).zfill(4))
        #numbering = numbering + 1

        #for (x,y,w,h) in cascades:
        #    image = cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)

        #time.sleep(0.1)
        #cv2.imshow('img',image)

if __name__ == '__main__':
    start()