#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("failed to open web-camera")
    exit()

red_upper_range = np.array([180, 255, 255])
red_lower_range = np.array([150, 140, 170])
green_upper_range = np.array([85, 255, 225])
green_lower_range = np.array([65, 240, 120])

morphology_kernel = np.ones((11, 11), np.uint8)
current_state = None


def stop():
    print("Stop!")
    
def start():
    print("Start!")

def detect_color(hsv_src, lower_range, upper_range, kernel):
    hsv_dst = cv2.inRange(hsv_src, lower_range, upper_range)
    dilated_hsv_dst = cv2.dilate(hsv_dst, kernel, iterations=1)
    contours, _ = cv2.findContours(dilated_hsv_dst, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def is_circular(contour):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return False
    circularity = 4 * np.pi * (area / (perimeter * perimeter))
    # 원형성의 범위를 좁혀서 사각형 제외
    return circularity > 0.85

def process_frame(src, red_kernel, green_kernel):
    gaussian_src = cv2.GaussianBlur(src, (11, 11), sigmaX=0, sigmaY=0)
    hsv_src = cv2.cvtColor(gaussian_src, cv2.COLOR_BGR2HSV)

    red_contours = detect_color(hsv_src, red_lower_range, red_upper_range, red_kernel)
    green_contours = detect_color(hsv_src, green_lower_range, green_upper_range, green_kernel)

    red_detected = any(is_circular(c) for c in red_contours if cv2.contourArea(c) >= 100)
    green_detected = any(is_circular(c) for c in green_contours if cv2.contourArea(c) >= 100)

    return red_detected, green_detected, red_contours, green_contours



while True:
    _, src = cap.read() 
    
    red_detected, green_detected, red_contours, green_contours = process_frame(src, morphology_kernel, morphology_kernel)

    # 이전과 다른 상태일 때만 호출되도록 
    if red_detected and current_state != 'red':
        current_state = 'red'
        stop()
    elif green_detected and current_state != 'green':
        current_state = 'green'
        start()
    elif not red_detected and not green_detected:
        current_state = None
 

    if cv2.waitKey(1) == ord('q'): break

cap.release()
cv2.destroyAllWindows()