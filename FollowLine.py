'''
此程式為2023AutoRace之循線程式
'''

import cv2
import serial
import numpy as np
import time

'''
    變數設定
'''
# 道路HSV遮色閥值
H_low = 0
S_low = 0
V_low = 235
H_high = 360
S_high = 255
V_high = 255

# 採樣間距
W_sampling_1 = 305
W_sampling_2 = 270
W_sampling_3 = 235
W_sampling_4 = 200

# 模式(0=雙循線,1=左循線,2=右循線)
mode = '0'
old_mode = '0'

# speed setting
base_spd = 100
error = 0
P = 0.7

'''
序列埠設定
'''
# COM_PORT = '/dev/ttyUSB0'  # 請自行修改序列埠名稱
COM_PORT = '/dev/ttyACM0'  # 請自行修改序列埠名稱
BAUD_RATES = 9600
ser = serial.Serial(COM_PORT, BAUD_RATES, timeout=0.1)

'''
    相機設定
'''
cap = cv2.VideoCapture(0)   # li_ubuntu:2
cap.set(3,5000)

def img_adjustment(img, Ksize=25, low_threshold=10, high_threshold=20, close_size=5):
    # 影像預處理
    # img = copy(img)
    img = cv2.resize(img,(640,360))
    # img = cv2.GaussianBlur(img, (11, 11), 0)
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    
    hsv_low = np.array([H_low,S_low,V_low])
    hsv_upper = np.array([H_high,S_high,V_high])
    mask = cv2.inRange(hsv,hsv_low,hsv_upper)

    # 線運算
    # Canny邊緣運算
    blur_gray = cv2.GaussianBlur(mask,(Ksize, Ksize), 0)
    canny_img = cv2.Canny(blur_gray, low_threshold, high_threshold)

    # 閉運算(解緩Canny斷線問題)
    kernel = np.ones((close_size,close_size),np.uint8)
    gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

    # cv2.imshow("gradient", gradient)
    # cv2.waitKey(1)
    return gradient

def Left_line(gradient):
    # left line limit
    L_min_300 = 0
    L_min_240 = 0
    L_min_180 = 0
    L_min_140 = 0
    
    # 霍夫變換
    lines = cv2.HoughLinesP(gradient,1,np.pi/180,8,5,2)
    if type(lines) == np.ndarray:
        for line in lines:
            x1,y1,x2,y2 = line[0]
            if ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_1:
                if ((x1+x2)/2)>L_min_300:
                    L_min_300 = int((x1+x2)/2)
            elif ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_2:
                if ((x1+x2)/2)>L_min_240:
                    L_min_240 = int((x1+x2)/2)
            elif ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_3:
                if ((x1+x2)/2)>L_min_180:
                    L_min_180 = int((x1+x2)/2)
            elif ((x1+x2)/2)<250 and ((y1+y2)/2)>W_sampling_4:
                if ((x1+x2)/2)>L_min_140:
                    L_min_140 = int((x1+x2)/2)   
    else:
        print("error")
        pass
    
    # 計算結果(車頭偏左負號)
    # L_min = 320-((L_min_300+L_min_240+L_min_180+L_min_140)/4)
    L_min = 320-((L_min_300+L_min_240)/2)
    
    return L_min,L_min_300,L_min_240,L_min_180,L_min_140
    
def Right_line(gradient):
    # 左右線極限X值(需重置)
    R_min_300 = 640
    R_min_240 = 640
    R_min_180 = 640
    R_min_140 = 640
    
    # 霍夫變換
    lines = cv2.HoughLinesP(gradient,1,np.pi/180,8,5,2)
    if type(lines) == np.ndarray:
        for line in lines:
            x1,y1,x2,y2 = line[0]
            # 右線
            if ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_1:
                if ((x1+x2)/2)<R_min_300:
                    R_min_300 = int((x1+x2)/2)
            elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_2:
                if ((x1+x2)/2)<R_min_240:
                    R_min_240 = int((x1+x2)/2)
            elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_3:
                if ((x1+x2)/2)<R_min_180:
                    R_min_180 = int((x1+x2)/2)
            elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_4:
                if ((x1+x2)/2)<R_min_140:
                    R_min_140 = int((x1+x2)/2)
    else:
        print("error")
        pass
    
    # R_min = ((R_min_300+R_min_240+R_min_180+R_min_140)/4)-320
    R_min = ((R_min_300+R_min_240)/2)-320
    
    return R_min,R_min_300,R_min_240,R_min_180,R_min_140
    
def get_offset_dis(L_min,R_min):
    offset_dis = int(L_min-R_min)
    # print(offset_dis)
    return offset_dis
    
def draw_line(img,L_min_300 = 640, L_min_240 = 640, L_min_180 = 640, L_min_140 = 640, R_min_300 = 0, R_min_240 = 0, R_min_180 = 0, R_min_140 = 0):
    pts = np.array([[L_min_300,(360+W_sampling_1)/2], [L_min_240,(W_sampling_1+W_sampling_2)/2], [L_min_180,(W_sampling_2+W_sampling_3)/2],[L_min_140,(W_sampling_3+W_sampling_4)/2]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    img = cv2.polylines(img, [pts], False,(255,200,0),3)

    pts = np.array([[R_min_300,(360+W_sampling_1)/2], [R_min_240,(W_sampling_1+W_sampling_2)/2], [R_min_180,(W_sampling_2+W_sampling_3)/2],[R_min_140,(W_sampling_3+W_sampling_4)/2]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    img = cv2.polylines(img, [pts], False,(255,200,0),3)
    
    return img
    # cv2.imshow("img", img)
    # cv2.waitKey(1)
    
def show_img(winname, img):
    cv2.imshow(winname, img)
    cv2.waitKey(1)
    
def motor_spd(offset_dis):
    # output speed
    L_spd = base_spd - ((offset_dis - error)*P)
    R_spd = base_spd + ((offset_dis - error)*P)
    
    # connect to opencr
    L_spd = str(L_spd)
    L_spd = L_spd+"L"
    L_spd = L_spd.encode('utf-8')
    ser.write(L_spd)
    
    R_spd = str(R_spd)
    R_spd = R_spd+"R"
    R_spd = R_spd.encode('utf-8')
    ser.write(R_spd)
 

def follow_double_line(img, gradient):
    L_min,L_min_300,L_min_240,L_min_180,L_min_140 = Left_line(gradient)
    R_min,R_min_300,R_min_240,R_min_180,R_min_140 = Right_line(gradient)
    offset_dis = get_offset_dis(L_min,R_min)
    line_img = draw_line(img, L_min_300, L_min_240, L_min_180, L_min_140, R_min_300, R_min_240, R_min_180, R_min_140)
    show_img("line", line_img)
    motor_spd(offset_dis)
    cv2.waitKey(1)
    
def follow_left_line(img, gradient):
    L_min,L_min_300,L_min_240,L_min_180,L_min_140 = Left_line(gradient)
    offset_dis = get_offset_dis(L_min,220)
    line_img = draw_line(img, L_min_300, L_min_240, L_min_180, L_min_140, 220, 220, 220, 220)
    show_img("line", line_img)
    motor_spd(offset_dis)
    cv2.waitKey(1)

def follow_right_line(img, gradient):
    R_min,R_min_300,R_min_240,R_min_180,R_min_140 = Right_line(gradient)
    offset_dis = get_offset_dis(R_min,265)
    line_img = draw_line(img, 265, 265, 265, 265, R_min_300, R_min_240, R_min_180, R_min_140)
    show_img("line", line_img)
    motor_spd(offset_dis)
    cv2.waitKey(1)

def print_mode_type(mode):
    if mode == '0':
        print("mode 0 : Double Line")

def main():
    while True:
        t = time.time()
        print_mode_type(mode)
        ret, img = cap.read()
        img = cv2.resize(img,(640,360))
        gradient = img_adjustment(img, Ksize=25, low_threshold=10, high_threshold=20, close_size=5)
        show_img("gradient", gradient)
        # L_min,L_min_300,L_min_240,L_min_180,L_min_140 = Left_line(gradient)
        # R_min,R_min_300,R_min_240,R_min_180,R_min_140 = Right_line(gradient)
        # offset_dis = get_offset_dis(L_min,R_min)
        # line_img = draw_line(img, L_min_300, L_min_240, L_min_180, L_min_140, R_min_300, R_min_240, R_min_180, R_min_140)
        # show_img("line", line_img)
        # motor_spd(offset_dis)
        cv2.waitKey(1)
        follow_double_line(img, gradient)
        # follow_left_line(img, gradient)
        # follow_right_line(img, gradient)
        print('t=', 1/(time.time()-t))

if __name__ == '__main__':
    main()
