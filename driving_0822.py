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
import time
import asyncio
from std_msgs.msg import Int32MultiArray

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
Offset = 250 #offset_y
Gap = 100 #wide_y
iteration = 1000
wide_x = Width-180
wide_y = 30
offset_x = 90
offset_y = 300
ultra_msg = [100 for i in range(8)]


cam = False
cam_debug = True

sub_f = 0
time_c = 0

async def print_angle(angle ):
    print(angle)

def img_callback(data):
    global image   
    global sub_f 
    global time_c

    sub_f += 1
    if time.time() - time_c > 1:
        time_c = time.time()
        sub_f = 0

    image = bridge.imgmsg_to_cv2(data, "bgr8")

def ultra_callback(data) :
    global ultra_msg
    ultra_msg = data.data

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
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []
    # print(slopes)

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        # print(slope)
        # print(line[0])
        
        if low_slope_threshold < abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])
    # print(slopes)

    # divide lines left to right
    left_lines = []
    right_lines = []
    horizental_line = False
    horizental_count = 0 
    
    th = -10
    # print(slopes)
    # print(len(slopes))
    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - th):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + th):
            right_lines.append([Line.tolist()])
        if -0.1 <= slope <= 0.1:
            # print(slopes)
            horizental_count+= 1
        
    if horizental_count >= 3 :
        horizental_line = True

    return left_lines, right_lines, horizental_line

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
        
        # print("b= {}, m={} pos={}".format(b,m,pos))
        

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
    low_threshold = 170
    high_threshold = 220
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold, kernel_size)
   
    # print(iteration)
    # file_path = '/home/pi/xycar_ws/src/driving/src/pic/{}.png'.format(iteration)
    # iteration+=1
    # cv2.imwrite(file_path,edge_img)

    # HoughLinesP
    #all_lines = cv2.HoughLinesP(edge_img, 0.7, math.pi/180, 8, 28, 2)
    
    all_lines = cv2.HoughLinesP(edge_img, 1 , math.pi/180, 1, minLineLength=5,maxLineGap=2)
    # all_lines = cv2.HoughLinesP(edge_img, 1 , math.pi/180, 30, 30, 7)
    # print(len(all_lines))

    # cv2.imshow("gray",gray)
    # cv2.imshow("roi ",roi)
    # cv2.imshow("Blurred Gray ",blur_gray)
    # cv2.imshow("edge image",edge_img)
    
    if cam:
        cv2.imshow('calibration', frame)
    # divide left, right lines
    if all_lines is None:
        return (Width)/2, (Width)/2, False
    # print(len(all_lines))
    left_lines, right_lines,horizental_line = divide_left_right(all_lines)
    
    # frame = draw_lines(frame,all_lines)
    # cv2.imshow('frame', frame)
    
    # cv2.waitKey(0)

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
    # cv2.waitKey(0)
    return lpos, rpos, len(all_lines), horizental_line

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

    # cv2.imshow('steer', img)
    file_path = '/home/pi/xycar_ws/src/driving/src/pic/{}.png'.format(iteration)
    iteration+=1
    # print(ultra_msg)
    cv2.imwrite(file_path,img)
    # cv2.waitKey(0)



    
    

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
    global iteration 
    start_time = 0
    end_time = 0
    end_time = 0
    cam_record = False
    exposure = 56
    cmd = "v4l2-ctl -d dev/videoCAM -c exposure_absolute={}".format(exposure)
    os.system(cmd)
    print("hello")
    rospy.init_node('auto_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    

    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    ultra_sub = None
    
    

    # rospy.wait_for_message("xycar_ultrasonic",Int32MultiArray)
    print("---------- Xycar C1 HD v1.0 ----------")
    time.sleep(3)

    sq = rospy.Rate(30)

    t_check = time.time()
    iteration += 1
    f_n = 0
    p_angle = 0
    flag = 0
    line_count = 0
    b_angle = 0
    b_error = 0
    ITerm = 0
    Cnt = 0
    speed = 20
    road_width = 0
    center = 0
    avoid_time = time.time() + 3.8
    turn_right = time.time()
    start_time = time.time()
    stop_time = time.time() + 1000000.5
    ultraStart = time.time()
    DRIVE_MODE = "BASIC"
    startlineStart = 0
    horizental_line = False
    angle = 0
    len_all_lines = 0
    passStart = 0
    last_ultra_1 = 100
    last_ultra_2 = 100
    current_ultra = 100
    last_ultra_left = 100
    current_ultra_left = 100

    if cam_record:
        fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        path = '/home/pi/xycar_ws/src'
        out = cv2.VideoWriter(os.path.join(path, 'test.avi'), fourcc, 25.0, (Width,Height))
    print("cam_recode ={}".format(cam_record))
    drive(0,0)
    
    error = 0
    p_angle = 0
    
    while not rospy.is_shutdown():
        
        # exposure += 7
        # cmd = "v4l2-ctl -d /dev/videoCAM -c exposure_absolute={}".format(exposure)
        # os.system(cmd)
        # while not image.size == (Width*Height*3):
        #     continue
        last_ultra_1 = last_ultra_2
        last_ultra_left = ultra_msg[1]
        last_ultra_2 = ultra_msg[2]
        
        current_ultra = ultra_msg[2]
        current_ultra_left = ultra_msg[1]
        f_n += 1
        if time.time() - ultraStart >= 3  and ultra_sub == None:
            ultra_sub = rospy.Subscriber("xycar_ultrasonic",Int32MultiArray,ultra_callback)
            asyncio.run(print_angle("ultra start!"))
            
        if ultra_sub != None :
            rospy.wait_for_message("xycar_ultrasonic",Int32MultiArray)
        if (time.time() - t_check) > 1:
            # print("fps : ", f_n)
            t_check = time.time()
            f_n = 0
        end_time = time.time()
        # print(end_time-start_time)
        if cam_record:
            out.write(image)
        draw_img = image.copy()
        # start_time= time.time()
        
        try:
            lpos, rpos, len_all_lines, horizental_line = process_image(draw_img)
        except:
            lpos, rpos, horizental_line = process_image(draw_img)
        #lpos, rpos, go = process_image(draw_img)
        # if time.time() > stop_time:
        #     print("stop_time", stop_time)
        #     line_count, flag, stop_time = stop(len_all_lines, flag, line_count, stop_time)
        #     print("stop_time", stop_time)
        #stop
        # print(horizental_line)
        if line_count==2:# 라인 카운트 2개시 정지 
            drive(0,0)
            cv2.waitKey(0)
            line_count = 0
        
        diff = rpos-lpos
        
        if road_width == 0 :
            road_width = diff
        # asyncio.run(print_angle("{}".format(len_all_lines)))
        # print("exposure = {} {} {}".format(exposure,len_all_lines,horizental_line))
   
        # if len_all_lines > 15 and DRIVE_MODE != "STARTLINE":
        #     line_count += 1
        #     DRIVE_MODE = "STARTLINE"
        #     startlineStart = time.time()
            
        #     # if time.time() -start_time > 10000:
        #     #     start_time = time.time()
        #     angle = 0
        #     speed = 20
            
        # elif DRIVE_MODE == 'STARTLINE' :
        #     if time.time() - startlineStart > 300:
        #         break
        #         DRIVE_MODE = "BASIC"
        #     angle = 0
        #     speed = 20
        # else:     
        # print(ultra_msg)
        # if 40 <= current_ultra <= 72 or 40 <= current_ultra_left <= 72:
        if 40 <= current_ultra <= 72 :
            # if last_ultra_1 < 60 and last_ultra_2 < 60 :
            # if 40 <= last_ultra_2 <= 80 or 40 <= last_ultra_left <= 80:
            if 40 <= last_ultra_2 <= 80 :
                
                # print("obstacle 1 {} -> {}".format(last_ultra_left,ultra_msg[1]))
                
                print("obstacle 2 {} -> {}".format(last_ultra_2,ultra_msg[2]))
                
                
                passStart = time.time()
                while time.time() - passStart < 0.8:
                    drive(70,23)
                

                passStart = time.time()
                while time.time() - passStart < 1.8:
                    drive(-70,25)
                # passStart = time.time()    
                # while time.time() - passStart < 0.75:
                #     drive(0,15)

                #passStart = time.time()
                #while time.time() - passStart < 1.2:
                #    drive(-60,25)
                passStart = time.time()
                while time.time() - passStart < 1:
                  drive(60,25)
                #drive(0,0)
            current_ultra = 100
            current_ultra_left = 100
            ultra_sub.unregister()
        
        speed = 27
        # if horizental_line and len_all_lines >= 20:
        if len_all_lines >= 40:
            # print(len_all_lines)
            print("horizental {} {}".format(horizental_line,len_all_lines))
            line_count += 1
            if line_count == 2:
                # time.sleep(2)
                drive(-15,20)
                break
            startlineStart = time.time()
            while time.time() - startlineStart < 2:
                drive(0,20)
            
        
        # if rpos < wide_x*0.6:
            
        #     angle = -60
            # asyncio.run(print_angle("lpos :{}, rpos : {} angle: {}".format(lpos,rpos,angle)))
        if lpos > wide_x * 0.4  : 
            angle = -47
            asyncio.run(print_angle("1. lpos :{}, rpos : {} angle: {}".format(lpos,rpos,angle)))
        elif lpos == 0 :

            # asyncio.run(print_angle("lpos = 0"))            
            if rpos > wide_x*0.8 or rpos < wide_x *0.66 :
                if time.time() - 1  < start_time:
                    angle = p_angle
                angle = -47
                # if p_angle <= -45:
                #     angle = p_angle - 0.1
                asyncio.run(print_angle("2. lpos :{}, rpos : {} angle: {}".format(lpos,rpos,angle)))
            else:       
                angle = p_angle
                # lpos = rpos - road_width 
                # center = (lpos + rpos) / 2
                # error = (center - wide_x/2)
                # angle, ITerm = pid_angle(ITerm, error, b_angle, b_error, Cnt)
                # steer_angle = angle * 0.4
                draw_steer(steer_angle)
                asyncio.run(print_angle("{}".format(iteration)))
                asyncio.run(print_angle("3. lpos :{}, rpos : {} angle: {}".format(lpos,rpos,angle)))
        elif rpos == Width :
            asyncio.run(print_angle("rpos = width"))     
            # angle = p_angle  
            
            if lpos < wide_x * 0.1 or lpos > wide_x * 0.4:
                angle = -47

                # if p_angle <= -45:
                #     angle = p_angle - 0.1
                asyncio.run(print_angle("4.lpos :{}, rpos : {} angle: {}".format(lpos,rpos,angle)))
            else:
                # rpos = lpos+road_width
                # center = (lpos + rpos) / 2
                # error = (center - wide_x/2)
                # angle, ITerm = pid_angle(ITerm, error, b_angle, b_error, Cnt)
                
                angle = p_angle
                steer_angle = angle * 0.4
                draw_steer(steer_angle)
                # drive(angle,speed)
                asyncio.run(print_angle("5. lpos :{}, rpos : {} angle: {}".format(lpos,rpos,angle)))
                
            #     global img
            #     cv2.imwrite('/home/pi/xycar_ws/src/driving/src/pic/number5.png',img)
            #     break
                
        else:
            # if rpos > wide_x * 0.9 or rpos < wide_x * 0.6:
            #     asyncio.run(print_angle("5-1. lpos :{}, rpos : {} angle: {}".format(lpos,rpos,angle)))
            #     rpos = lpos + road_width
            
            center = (lpos + rpos) / 2
            error = (center - wide_x/2)
            angle, ITerm = pid_angle(ITerm, error, b_angle, b_error, Cnt)
            asyncio.run(print_angle("6. lpos :{}, rpos : {} angle: {}".format(lpos,rpos,angle)))
        # if time.time() - start_time < 1.5:
        #     angle = 0
        #     asyncio.run(print_angle("lpos :{}, rpos : {} angle: {}".format(lpos,rpos,angle)))



        # if wide_x/2 - 10 < center <wide_x/2+10:
        #     angle = 0
        #     speed = 25
        # elif rpos < wide_x*0.6:
            
        #     angle = -65
        # elif lpos == 0 :
        
        #     if rpos > wide_x*0.8 :
        #         angle = -65
        #     else:       
        #         lpos = rpos - road_width 
        #     center = (lpos + rpos) / 2
        #     error = (center - wide_x/2)
        #     angle, ITerm = pid_angle(ITerm, error, b_angle, b_error, Cnt)
            

        # cv2.putText(img,'lpos={} rpos={} center={} angle={}'.format(lpos,rpos,center,angle),(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
        # steer_angle = angle * 0.4
        # draw_steer(steer_angle)

        # asyncio.run(print_angle("lpos :{}, rpos : {} angle: {}".format(lpos,rpos,angle)))
        
        # if angle >= 10 or angle <= -10:
        #     speed = 15
        # else:
        #     speed = 15
        # print(speed)
        
        drive(angle, speed)
        # time.sleep(1)
            
        cv2.waitKey(1)
        #sq.sleep()
        b_angle =angle
        b_error = error
        p_angle = angle


if __name__ == '__main__':
    start()
