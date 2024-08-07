#!/usr/bin/env python

import rospy, rospkg, time
import numpy as np
import cv2, math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
motor = None
Width = 640
Height = 480
Offset = 300 #offset_y
Gap = 100 #wide_y
iteration = 1000
wide_x = Width
wide_y = 30
offset_x = 0
offset_y = 300

cam = False
cam_debug = True

sub_f = 0
time_c = 0

def img_callback(data):
    global image   
    global sub_f 
    global time_c

    sub_f += 1
    if time.time() - time_c > 1:
        time_c = time.time()
        sub_f = 0

    image = bridge.imgmsg_to_cv2(data, "bgr8")

# publish xycar_motor msg
def drive(Angle, Speed): 
    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed

    motor.publish(motor_msg)

# draw lines
def draw_lines(img, lines):
    global Offset
    global offset_x, offset_y
    for line in lines:
        x1, y1, x2, y2 = line[0]
        img = cv2.line(img, (x1+offset_x, y1+offset_y), (x2+offset_x, y2+offset_y), (0, 255, 0), 2)
        # img = cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return img

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2
    center = int(center)

    # cv2.rectangle(img, (lpos - 2, 7 + offset),
    #                    (lpos + 2, 12 + offset),
    #                    (0, 0, 0), 2)
    # cv2.rectangle(img, (rpos - 2, 7 + offset),
    #                    (rpos + 2, 12 + offset),
    #                    (255, 0, 0), 2)
    # cv2.rectangle(img, (center-2, 7 + offset),
    #                    (center+2, 12 + offset),
    #                    (0, 255, 0), 2)    
    # cv2.rectangle(img, (157, 7 + offset),
    #                    (162, 12 + offset),
    #                    (0, 0, 255), 2)
    return img

# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 20

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        
        if low_slope_threshold < abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []
    th = -10

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - th):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + th):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines

# get average m, b of line, sum of x, y, mget lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap, cam_debug

    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    
    m = 0
    b = 0

    if size != 0:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)
        # print("x_avg {} y_avg {}".format(x_avg,y_avg))
        
        m = m_sum / size
        b = y_avg - m * x_avg

    if m == 0 and b == 0:
        if left:
            pos = 0
        elif right:
            pos = Width
    else:
        y = wide_y/2

        pos = (y - b) / m
        
        print("b= {}, m={} pos={}".format(b,m,pos))
        

        if cam_debug:
            b += offset_y
            xs = (Height - b) / float(m)
            xe = ((Height/2) - b) / float(m)
            

            cv2.line(img, (int(xs)+offset_x, int(Height)), (int(xe)+offset_x, int(Height/2)), (255, 0,0), 3)

    return img, int(pos)

def stop(all_lines, flag, line_count, stop_time):#정지선 인식 함수
    line_len = all_lines
    print("all_lines",line_len)
    # 49
    # flag: 0 이면 나오게 함
    if (line_count == 1) and (line_len > 30): #출발후 1바퀴 돌고-> 1번째 바퀴 완주전 2번-> 2번째 바퀴 인식하고 정지
        flag = 0
        line_count = 2
        return line_count, flag, stop_time
    
    if (line_len > 30): #출발후  1바퀴 돌고
        flag = 1
        line_count = 1
        print("flag up")
        print("flag up")
        stop_time = time.time() + 10.5
        print("stop_time:", stop_time, " time:", time.time())
        


    return line_count, flag, stop_time

# show image and return lpos, rpos
def process_image(frame):
    global Width
    global wide_x
    global wide_y
    global Offset, Gap
    global offset_x,offset_y
    global cam, cam_debug, img
    global iteration

    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    roi = gray[offset_y : offset_y+wide_y, offset_x : offset_x+wide_x]
    # roi = gray[250:250+80,160: 480]
    # cv2.imshow("original",frame)
    
    
    # blur
    kernel_size = 5
    standard_deviation_x = 3    #Kernel standard deviation along X-axis
    blur_gray = cv2.GaussianBlur(roi, (kernel_size, kernel_size), standard_deviation_x)
    

    # canny edge
    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold, kernel_size)
    print(iteration)
    # file_path = '/home/pi/xycar_ws/src/driving/src/pic/{}.png'.format(iteration)
    # iteration+=1
    # cv2.imwrite(file_path,edge_img)

    # HoughLinesP
    #all_lines = cv2.HoughLinesP(edge_img, 0.7, math.pi/180, 8, 28, 2)
    all_lines = cv2.HoughLinesP(edge_img, 1 , math.pi/180, 5, 30, 2)
    # cv2.imshow("gray",gray)
    # cv2.imshow("roi ",roi)
    # cv2.imshow("Blurred Gray ",blur_gray)
    # cv2.imshow('Edge image',edge_img)
    # cv2.waitKey(0)
    if cam:
        cv2.imshow('calibration', frame)
    # divide left, right lines
    if all_lines is None:
        return (Width)/2, (Width)/2, False
    left_lines, right_lines = divide_left_right(all_lines)
    # print("left : {}, right :{}".format(len(left_lines),len(right_lines)))

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)

    if cam_debug:
        # draw lines
        frame = draw_lines(frame, left_lines)
        frame = draw_lines(frame, right_lines)
        # frame = cv2.line(frame, (115, 117), (205, 117), (0,255,255), 2)

        # draw rectangle
        frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
        frame = cv2.rectangle(frame, (offset_x+wide_x, offset_y), (offset_x, offset_y+wide_y), (255, 202, 204), 2)
    
    img = frame        
    return lpos, rpos, len(all_lines), True

def draw_steer(steer_angle):
    global Width, Height, img, iteration
    # img = cv_image
    arrow = cv2.imread('/home/pi/xycar_ws/src/driving/src/steer_arrow.png')

    origin_Height = arrow.shape[0]
    origin_Width = arrow.shape[1]
    steer_wheel_center = origin_Height * 0.74
    arrow_Height = Height/2
    arrow_Width = (arrow_Height * 462)/728
    arrow_Height = int(arrow_Height)
    arrow_Width = int(arrow_Width)

    matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (-steer_angle) * 1.5, 0.7)    
    arrow = cv2.warpAffine(arrow, matrix, (origin_Width+60, origin_Height))
    arrow = cv2.resize(arrow, dsize=(arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)

    gray_arrow = cv2.cvtColor(arrow, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

    arrow_roi = img[arrow_Height: int(Height), int(Width/2 - arrow_Width/2) : int(Width/2 + arrow_Width/2)]
    arrow_roi = cv2.add(arrow, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow)
    img[int(Height - arrow_Height): int(Height), int(Width/2 - arrow_Width/2): int(Width/2 + arrow_Width/2)] = res

    cv2.imshow('steer', img)
    file_path = '/home/pi/xycar_ws/src/driving/src/pic/{}.png'.format(iteration)
    iteration+=1
    cv2.imwrite(file_path,img)
    cv2.waitKey(0)


    
    

def pid_angle(ITerm, error, b_angle, b_error, Cnt):
    angle = 0
    Kp = 0.92 #0.5 good / if Kp high -> loss decrease+faster respone but incur overshoot
    Ki = 0.00065 #0.0001 good #0.0002 / if Ki high 
    #-> accumulated loss increase faster+faster response but incur overshoot
    Kd = 0.0925 #1.0 good #2.0/ decrease the vibration
            # if Kd high -> decrease overshoot but when the signal changes rapidly
            # it can make the system destroy
    dt = 1
    
    # 속도 높이면 PID 제어 다시 해줘야 하는데 정착시간까지 반응이 안 일어나기 때문에
    # I 제어기가 필요하지 않음 -> PD 제어기 사용하는 경우 많음.

    PTerm = Kp * error
    ITerm += Ki * error * dt
    derror = error - b_error
    DTerm = Kd * (derror / dt)
    #angle = PTerm + ITerm + DTerm
    angle = PTerm + DTerm

    return angle, ITerm

def start():
    global motor
    global image
    global Width, Height
    cam_record = False
    print("hello")
    rospy.init_node('auto_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    print("---------- Xycar C1 HD v1.0 ----------")
    time.sleep(3)

    sq = rospy.Rate(30)

    t_check = time.time()
    f_n = 0
    p_angle = 0
    flag = 0
    line_count = 0
    b_angle = 0
    b_error = 0
    ITerm = 0
    Cnt = 0
    speed = 15
    avoid_time = time.time() + 3.8
    turn_right = time.time()
    stop_time = time.time() + 1000000.5

    if cam_record:
        fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        path = '/home/pi/xycar_ws/src/base/cam_record'
        out = cv2.VideoWriter(os.path.join(path, 'test.avi'), fourcc, 25.0, (Width,Height))
    print("cam_recode ={}".format(cam_record))
    drive(0,0)
    
    error = 0
    p_angle = 0
    while not rospy.is_shutdown():
        
        # while not image.size == (Width*Height*3):
        #     continue
        
        
        f_n += 1
        if (time.time() - t_check) > 1:
            # print("fps : ", f_n)
            t_check = time.time()
            f_n = 0
        if cam_record:
            out.write(image)
        draw_img = image.copy()
        
        try:
            lpos, rpos, len_all_lines, go = process_image(draw_img)
        except:
            lpos, rpos, go = process_image(draw_img)
        #lpos, rpos, go = process_image(draw_img)
        if time.time() > stop_time:
            print("stop_time", stop_time)
            line_count, flag, stop_time = stop(len_all_lines, flag, line_count, stop_time)
            print("stop_time", stop_time)
         #stop
        if (line_count==2):# 라인 카운트 2개시 정지 
            drive(0,0)
            cv2.waitKey(0)
            line_count = 0

        diff = rpos-lpos
        # print(lpos, rpos, center, diff)

        # if diff > 135 and diff < 142:
        #     print("straight")
        # else:
        #     print("curve")

        
        
        # if(lpos == 0):
        #     print("lpos error")
        #     lpos = rpos - 350
        #     if lpos < 0:
        #         lpos = 0
        # if(rpos > lpos+350):
        #     print("rpos error")
        #     rpos=lpos+280
        speed = 15
   


        if int(lpos) == 0 and int(rpos) >= Width:
            angle =p_angle
        elif rpos > 500:
            angle = p_angle
        else:
            
            if center >= 300 and center <= 360 :
                angle = 0
            else:
                if lpos == 0:
                    lpos = rpos - 130 
                center = (lpos + rpos) / 2
            
                cv2.putText(img,'lpos={} rpos={} center={}'.format(lpos,rpos,center),(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
                # angle = -(Width/2 - center)
                error = (center - wide_x/2)
                angle, ITerm = pid_angle(ITerm, error, b_angle, b_error, Cnt)
                p_angle = angle
                print(lpos, rpos, center, diff)


#        if lpos == 0 and rpos == 320:
#            angle = 70
#            drive(angle, 5)

        steer_angle = angle * 0.4
        draw_steer(steer_angle)
        print("angle :{}".format(angle))
        if angle >= 10 or angle <= -10:
            speed = 15
        else:
            speed = 15
        drive(angle, speed)
            
        cv2.waitKey(1)
        #sq.sleep()
        b_angle =angle
        b_error = error


if __name__ == '__main__':
    start()
