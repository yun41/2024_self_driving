#!/usr/bin/env python
# -*- coding: utf-8 -*- 1
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, rospy, time, math, os
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
from cv_bridge import CvBridge

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None  # 모터 노드 변수
Fix_Speed = 12  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지 
ultra_msg = None  # 초음파 데이터를 담을 변수
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
motor_msg = xycar_motor()  # 모터 토픽 메시지
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
View_Center = WIDTH/2

#=============================================
# 콜백함수 - USB 카메라 토픽을 받아서 처리하는 콜백함수
#=============================================
def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 콜백함수 - 초음파 토픽을 받아서 처리하는 콜백함수
#=============================================
def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data

#=============================================
# 모터 토픽을 발행하는 함수 
#=============================================
def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)
		
#=============================================
# 카메라의 Exposure 값을 변경하는 함수 
# 입력으로 0~255 값을 받는다.
#=============================================
def cam_exposure(value):
    command = 'v4l2-ctl -d /dev/videoCAM -c exposure_absolute=' + str(value)
    os.system(command)
    
#=============================================
# 신호등의 파란불을 체크해서 True/False 값을 반환하는 함수
#=============================================
def check_traffic_sign():
    MIN_RADIUS, MAX_RADIUS = 15, 25
    
    # 원본이미지를 복제한 후에 특정영역(ROI Area)을 잘라내기
    # 원본 이미지에서 ROI 영역만큼 잘라서 roi_img에 담음 
    # roi_img 칼라 이미지를 회색 이미지로 바꾸고 노이즈 제거를 위해 블러링 처리를 함  
    # Hough Circle 함수를 이용해서 이미지에서 원을 (여러개) 찾음 
    
    # 이미지에서 정확하게 3개의 원이 발견됐다면 신호등 찾는 작업을 진행  
    # 3개 중에서 세번째 원이 가장 밝으면 (파란색 신호등) True 리턴 
    return True
    # 첫번째나 두번째 원이 가장 밝으면 (파란색 신호등이 아니면) False 반환 
    return False
   
    # 원본 이미지에서 원이 발견되지 않았다면 False 리턴   
    return False
       
#=============================================
# 초음파 센서를 이용해서 벽까지의 거리를 알아내서
# 벽과 충돌하지 않으며 주행하도록 모터로 토픽을 보내는 함수
#=============================================
def sonic_drive():
    global new_angle, new_speed

    # 왼쪽 벽이 오른쪽 벽보다 멀리 있으면, 왼쪽으로 주행
    if (ultra_msg[1] > ultra_msg[3]):
        new_angle = -50

    # 왼쪽 벽보다 오른쪽 벽이 멀리 있으면, 오른쪽으로 주행
    elif (ultra_msg[1] < ultra_msg[3]):
        new_angle = 50

    # 위 조건들에 해당하지 않는 경우라면 직진 주행
    else:
        new_angle = 0

    # 모터에 주행명령 토픽을 보낸다
    new_speed = Fix_Speed
    drive(new_angle, new_speed)       

#=============================================
# 정지선이 있는지 체크해서 True/False 값을 반환하는 함수
#=============================================
def check_stopline():
    global stopline_num

    # image(원본이미지)의 특정영역(ROI Area)을 잘라내기
    # HSV 포맷으로 변환하고 V채널에 대해 범위를 정해서 흑백이진화 이미지로 변환
    # 흑백이진화 이미지에서 특정영역을 잘라내서 정지선 체크용 이미지로 만들기
    # 정지선 체크용 이미지에서 흰색 점의 개수 카운트하기
    
    # 사각형 안의 흰색 점이 기준치 이상이면 정지선을 발견한 것으로 한다
    return True
    
#=============================================
# 카메라 이미지에서 차선을 찾아 그 위치를 반환하는 함수
#=============================================
def lane_detect():

    global image
    prev_x_left = 0
    prev_x_right = WIDTH

    img = image.copy() # 이미지처리를 위한 카메라 원본이미지 저장
    display_img = img  # 디버깅을 위한 디스플레이용 이미지 저장

    #=========================================    
    # img(원본이미지)의 특정영역(ROI Area)을 잘라내기
    # 잘라낸 이미지에서 HoughLinesP 함수를 사용하여 선분들을 찾음
    #=========================================
    # 왼쪽 차선에 해당하는 선분과 오른쪽 차선에 해당하는 선분을 구분함.
    #=========================================
    # 왼쪽/오른쪽 차선에 해당하는 선분들의 데이터를 적절히 처리해서 
    # 왼쪽차선의 대표직선과 오른쪽차선의 대표직선을 각각 구함.
    #=========================================
    # 기준선(수평선)과 대표직선과의 교점인 x_left와 x_right를 찾음.
    #=========================================

    x_left = 100
    x_right = 500
    return True, x_left, x_right

#=============================================
# 실질적인 메인 함수 
#=============================================
def start():

    global motor, ultra_msg, image 
    global new_angle, new_speed
   
    TRAFFIC_SIGN = 2
    SENSOR_DRIVE = 3
    LANE_DRIVE = 4
    AR_DRIVE = 5
    PARKING = 7
    FINISH = 9
		
    # 처음에 어떤 미션부터 수행할 것인지 여기서 결정합니다. 
    drive_mode = TRAFFIC_SIGN
    
    cam_exposure(150)  # 카메라의 Exposure 값을 변경
    
    #=========================================
    # 노드를 생성하고, 구독/발행할 토픽들을 선언합니다.
    #=========================================
    rospy.init_node('Track_Driver')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/",Image,usbcam_callback, queue_size=1)

    #=========================================
    # 발행자 노드들로부터 첫번째 토픽들이 도착할 때까지 기다립니다.
    #=========================================
    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")
    rospy.wait_for_message("xycar_ultrasonic", Int32MultiArray)
    print("UltraSonic Ready ----------")

    print("======================================")
    print(" S T A R T    D R I V I N G ...")
    print("======================================")
	
    drive(0,0)
    time.sleep(5)

    print ("----- Traffic Sign Check Start... -----")

    #=========================================
    # 메인 루프 
    #=========================================
    while not rospy.is_shutdown():

        # ======================================
        # 출발선에서 신호등을 찾습니다. 
        # 일단 정차해 있다가 파란색 불이 켜지면 출발합니다.
        # SENSOR_DRIVE 모드로 넘어갑니다.  
        # ======================================
        while drive_mode == TRAFFIC_SIGN:
		
            # 앞에 있는 신호등에 파란색 불이 켜졌는지 체크합니다.  
            result = check_traffic_sign()
			
            if (result == True):
                # 신호등이 파란불이면 SENSOR_DRIVE 모드로 넘어갑니다.
                drive_mode = SENSOR_DRIVE
                cam_exposure(100)  # 카메라의 Exposure 값을 변경
                print ("----- SENSOR_DRIVE Start... -----")

        time.sleep(5)

        # ======================================
        # 초음파센서로 미로주행을 진행합니다.
        # 정지선이 보이면 차선인식주행 LANE_DRIVE 모드로 넘어갑니다. 
        # ======================================
        while drive_mode == SENSOR_DRIVE:

            # 초음파센서를 이용해서 미로주행을 합니다. 
            sonic_drive()  

            # 앞에 정지선이 있는지 체크합니다.  
            result = check_stopline() 

            if (result == True):
                # 정지선이 발견되면 차량을 정차시키고 LANE_DRIVE 모드로 넘어갑니다.
                drive(0,0)
                drive_mode = LANE_DRIVE  
                cam_exposure(100)  # 카메라의 Exposure 값을 변경
                print ("----- LANE_DRIVE Start... -----")
                         
        time.sleep(5)
     
        # ======================================
        # 차선을 보고 주행합니다. 
        # ======================================
        while drive_mode == LANE_DRIVE:
		
            # 카메라 이미지에서 차선의 위치를 알아냅니다. 
            found, x_left, x_right = lane_detect()
			
            if found:
                # 차선인식이 됐으면 차선의 위치정보를 이용해서 핸들 조향각을 결정합니다. 
                x_midpoint = (x_left + x_right) // 2 
                new_angle = (x_midpoint - View_Center) / 2
                new_speed = Fix_Speed
                drive(new_angle, new_speed)  
				
            else:
                # 차선인식이 안됐으면 기존 핸들값을 사용하여 주행합니다. 	
                drive(new_angle, new_speed)
                  
        time.sleep(5)

        # ======================================
        # 주행을 끝냅니다. 
        # ======================================
        if drive_mode == FINISH:
           
            # 차량을 정지시키고 모든 작업을 끝냅니다.
            stop_car(10) # 1초간 정차 
            print ("----- Bye~! -----")
            return            

#=============================================
# 메인함수를 호출합니다.
# start() 함수가 실질적인 메인함수입니다.
#=============================================
if __name__ == '__main__':
    start()
