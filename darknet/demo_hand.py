#sudo usermod -aG dialout $USER
#sudo apt-get install libxcb-xinerama0
#pip install numpy
#pip install scipy
#pip install PyQt5
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from threading import Thread
from scipy.optimize import root,fsolve,least_squares
import numpy as np
import math
import time
import cv2
import sys
import traceback
import serial
import serial.tools.list_ports
import detector1
import detector2

class MainWindow(QMainWindow):
    def initial_hand(self,flag=False):
        def rotate_zeros():
          ang = [0,0,0,0]
          speed = 2000
          self.serial.write(str.encode(f'#0P{self.deg2period(0,ang[0])}S{speed}#1P{self.deg2period(1,ang[1])}S{speed}#2P{self.deg2period(2,ang[2])}S{speed}#3P{self.deg2period(3,ang[3])}S{speed}#5P{self.deg2period(5,150)}S{speed}\r\n'))
          self.pre_angle = ang
        self.serial.write(str.encode('QP0\r\n'))
        res = self.serial.read()
        if res == b'' or res == b'\x00':
            i0 = 2500
            i1 = 500
            i2 = 1673
            i3 = self.deg2period(3,0)
            i4 = 1500
            i5 = 1550
            cmd = f'#0P{i0}S500#1P{i1}S500#2P{i2}S500#3P{i3}S500#4P{i4}S500#5P{i5}S500\r\n'
            self.serial.write(str.encode(cmd))
            rotate_zeros()
        if flag:
            print('init')
            rotate_zeros()
    def get_board(self,contours,w0,h0,start_y):
        min_x = w0
        max_x = 0
        min_y = h0
        max_y = 0
        length = len(contours)
        for i in range(length):
            c = contours[i]
            x, y, w, h = cv2.boundingRect(c)
            if w/w0>0.3 and w/h>10:

                #print(w/h)
                #print(w,h)
                if x<min_x:
                    min_x = x
                if y<min_y:
                    min_y = y
                if x+w>max_x:
                    max_x = x+w
                if y+h>max_y:
                    max_y = y+h
        
        return [min_x,min_y+start_y,max_x,max_y+start_y]

    def get_roi(self,contours,w0,h0):
        min_x = w0
        max_x = 0
        min_y = h0
        max_y = 0
        length = len(contours)
        for i in range(length):
            c = contours[i]
            x, y, w, h = cv2.boundingRect(c)
            if 0.9>w/w0>0.3 and w/h<3:
                if x<min_x:
                    min_x = x
                if y<min_y:
                    min_y = y
                if x+w>max_x:
                    max_x = x+w
                if y+h>max_y:
                    max_y = y+h
        
        return [min_x,min_y,max_x,max_y]
    def get_top_circle(self,contours,min_x,max_x,min_y,ratio,rect_h,img):
        circles = []
        length = len(contours)
        for i in range(length):
          c = contours[i]
          x, y, w, h = cv2.boundingRect(c)
          if 0 != x and 0!=y:
            if 0.6<=w/h<=1.4 and 0.2>w/(max_x-min_x)>=0.05:
              nonzero = cv2.countNonZero(img[y:y+h,x:x+w])
              if nonzero/w/h>0.2:
                p_x = x+w//2
                p_y = y+h//2
                obj_x = int(p_x*ratio)
                obj_y = int((rect_h*181/400-p_y)*ratio)
                circles +=  [[obj_x,obj_y,(p_x+min_x,p_y+min_y),(w+h)//4,int((w+h)//2*ratio)]]
        return circles
    def get_side_circle(self,contours,min_x,max_x,min_y,border_y,ratio_x,ratio_y,rect_h,border_w,roi):
        circles = []
        length = len(contours)
        for i in range(length):
            c = contours[i]
            x, y, w, h = cv2.boundingRect(c)
            if 0 != x and 0!=y: # and len(c)>5:
                #(c_x,c_y), size, _ = cv2.fitEllipse(c)
                p_x = x+w//2
                p_y = y+h//2
                b_width = (max_x-min_x)
                ball_side = []
                obj_y = int((border_y-p_y)*ratio_y)
                if 0.6<=w/h<=1.4 and 0.2>w/b_width>=0.03 and cv2.countNonZero(roi[y:y+h,x:x+w])/(w*h)>0.5 and obj_y<30: #and 1.4>size[0]/size[1]>0.6 and 0<abs(c_x-p_x)/b_width<0.02 and 0<abs(c_y-p_y)/b_width<0.02
                    #print(w,h)
                    obj_x = int((border_w-p_x)*ratio_x)
                    #print(obj_x,abs(c_x-p_x),abs(c_y-p_y),cv2.countNonZero(roi[y:y+h,x:x+w]))
                    ball_side += [[obj_x,obj_y]]
                    circles +=  [[obj_x,obj_y,(p_x+min_x,p_y+min_y),(w+h)//4]]
                if len(ball_side) != 0:
                    self.ball_side = ball_side
        return circles

    def resizeEvent(self, event):
        self.width = self.frameGeometry().width()
        self.height = self.frameGeometry().height()
        QMainWindow.resizeEvent(self, event)
    def set_data(self,*args):
        self.data.put(args)
    def lock_board(self):
        self.lock_board_flag = True
        self.board_max_y = 0
    def cal_jaw_d(self,theta2):
        r1 = 23
        r2 = 30
        pre_rotate_ang = 65
        jaw_len = 30
        return 2*(15-r1*math.cos(pre_rotate_ang/180*math.pi)-r2*math.cos(theta2/180*math.pi)-jaw_len*math.cos((pre_rotate_ang+15)/180*math.pi))
    def cal_jaw_ang(self,d):
        def get_jaw_ang(ang,*args, **kwargs):
          d = args[0]
          return d-self.cal_jaw_d(ang)
        ang = least_squares(fun=get_jaw_ang,loss='soft_l1',tr_solver='exact',x0=150,args=(d/5*1,))
        return ang.x[0]
    def catch_ball(self):
        a=89 #89
        b=105
        c=133
        d=170
        bending_height = 0 #單位:mm

        self.small_r = 10
        """
        i0 = 13.95 #底座上面第一節 -90~(90-22) 防止撞到
        i1 = 16.65 #底座上面第二節 -90~90
        i2 = -0.9 #底座上面第三節 -90~90 
        i3 = -6.57 #底座 -90~90
        """
        #i4
        #i5#950最小1500最大
        d2r = math.pi/180
        def get_hand(t0,t1,t2,t3):
          ang0 = t0
          ang1 = t1
          ang2 = t2
          ang3 = t3
          alpha = ang0 * d2r
          beta = (-ang0+ang1) * d2r
          gamma = (-ang0+ang1+ang2) *d2r
          delta = ang3 * d2r
          #print('alpha',alpha/d2r,beta/d2r,gamma/d2r,delta/d2r)
          z=(a+b*math.cos(alpha)+c*math.sin(beta)+d*math.sin(gamma)) - bending_height
          r=b*math.sin(alpha)+c*math.cos(beta)+d*math.cos(gamma)
          #print(r)
          x=r*math.cos(delta) + self.small_r*math.sin(delta)
          y=r*math.sin(delta) - self.small_r*math.cos(delta)
          return [x,y,z]
        def hand_func(vars,*args, **kwargs):
          temp = get_hand(vars[0],vars[1],vars[2],args[3])
          p = [(temp[0]-args[0]),(temp[1]-args[1]),(temp[2]-args[2])]
          #print('p',p)
          return p

        def solve_angle(x,y,z):
          delta_theta = math.atan2(x,-y)
          r = (x**2+y**2)**(1/2)
          theta = math.atan2(r,self.small_r)/d2r
          R = self.small_r/math.cos(theta*d2r)
          delta_theta /= d2r #-90~90 deg
          delta = delta_theta - theta
          #print('delta_theta',delta_theta,'theta',theta,'delta',delta)
          pre_angle = (*self.pre_angle[0:3],)
          #print(pre_angle)
          t3 = delta
          #print(t3)
          sol3_root = least_squares(fun=hand_func,loss='soft_l1',tr_solver='exact',jac='3-point',x0=pre_angle,args=(x, y, z, t3),bounds=(self.min_ang[0:3],self.max_ang[0:3]))
          print(sol3_root)
          return [*sol3_root.x,t3]
        def dynamic_catch_ball_at_x_y_z():
          def slide_ang(ang,pre_ang):
            limit_move_ang = 3
            if (ang-pre_ang) > limit_move_ang:
              return pre_ang + limit_move_ang
            elif (ang-pre_ang) < -limit_move_ang:
              return pre_ang - limit_move_ang
            else:
              return ang
          def get_distance(p1,p2):
            d = 0
            for x0,x1 in zip(p1,p2):
              d += (x0-x1)**2
            return d**(1/2)
          def change_small_r(x,y):
            ang = math.atan2(y,x)/math.pi*180
            print('ang-------------------\n',ang)
            """
            if 5>=ang>=-5:
              self.small_r = 15
            elif -5>=ang>=-15:
              self.small_r = 32
            elif -15>ang>-35:
              self.small_r = 40
            elif -60<=ang<-35:
              self.small_r = 25
            elif ang<-60:
              self.small_r = 10
            elif 20>=ang>5:
              self.small_r = 5
            elif 30>=ang>20:
              self.small_r = -5
            elif 45>=ang>30:
              self.small_r = -20
            elif 60>=ang>45:
              self.small_r = -15
            elif ang>90:
              self.small_r = -30
            """
            #self.small_r = 0
          self.stop_flag = False
          pre_angle = self.pre_angle
          pre_x,pre_y,pre_z = get_hand(*pre_angle[0:4])
          print('pre',pre_x,pre_y,pre_z)
          key = list(self.ball_save_dict.keys())[0]
          x,y,z,D,v,a,image_cord,image_R,ratio = self.ball_save_dict[key]
          #change_small_r(x,y)
          D_h = 3
          if pre_z<z+D*D_h:
            print('go to top')
            ang = solve_angle(pre_x,pre_y,z+D*1.5)
            speed = '2000'
            cmd_p0 = f'#0P{self.deg2period(0,ang[0])}S{speed}#1P{self.deg2period(1,ang[1])}S{speed}#2P{self.deg2period(2,ang[2])}S{speed}#3P{self.deg2period(3,ang[3])}S{speed}#5P{self.deg2period(5,150)}S{speed}\r\n'
            self.serial.write(str.encode(cmd_p0))
            self.wait_rotation()
          print('start track')
          self.serial.write(str.encode(f'#5P{self.deg2period(5,150)}S2000\r\n'))
          self.wait_rotation()
          while 1:
            if self.stop_flag:
              break
            x,y,z,D,v,a,image_cord,image_R,ratio = self.ball_save_dict[key]
            #change_small_r(x,y)
            z += D*D_h
            #y -= 30
            ang = solve_angle(x,y,z)
            ang_to_move = [slide_ang(ang_move,pre_angle) for ang_move,pre_angle in zip(ang,self.pre_angle[0:4])]
            self.pre_angle = [*ang_to_move,0,0]
            #speed = '1000' #works very perfect
            speed = '2000'
            cmd_p0 = f'#0P{self.deg2period(0,ang_to_move[0])}S{speed}#1P{self.deg2period(1,ang_to_move[1])}S{speed}#2P{self.deg2period(2,ang_to_move[2])}S{speed}#3P{self.deg2period(3,ang_to_move[3])}S{speed}S{speed}\r\n'
            self.serial.write(str.encode(cmd_p0))
            x_new,y_new,z_new = get_hand(ang_to_move[0],ang_to_move[1],ang_to_move[2],ang_to_move[3])
            print('to_go',x,y,z,D)
            print('cal',x_new,y_new,z_new,D)
            diff = get_distance((x_new,y_new,z_new),(x,y,z))
            self.wait_rotation()
            if diff <0.5:
              print(ang_to_move)
              print(diff)
              print('finish')
              ang = solve_angle(x,y,z-D*D_h)
              print('ang',ang)
              #time.sleep(0.2) #works very perfect
              cmd_p0 = f'#0P{self.deg2period(0,ang[0])}S{speed}#1P{self.deg2period(1,ang[1])}S{speed}#2P{self.deg2period(2,ang[2])}S{speed}#3P{self.deg2period(3,ang[3])}S{speed}S{speed}\r\n'
              self.serial.write(str.encode(cmd_p0))
              self.wait_rotation()
              self.serial.write(str.encode(f'#5P{self.deg2period(5,self.cal_jaw_ang(D))}S500\r\n'))
              self.ball_save_dict.pop(key)
              self.wait_rotation()
              time.sleep(0.2)
              

              ang = solve_angle(x,y,z+D*D_h*2)
              
              #ang = [-70,-35,-50,150]
              
              speed = '2000'
              cmd_p0 = f'#0P{self.deg2period(0,ang[0])}S{speed}#1P{self.deg2period(1,ang[1])}S{speed}#2P{self.deg2period(2,ang[2])}S{speed}#3P{self.deg2period(3,ang[3])}S{speed}\r\n'
              self.serial.write(str.encode(cmd_p0))
              #self.wait_rotation()
              #time.sleep(1)
              #self.serial.write(str.encode(f'#5P{self.deg2period(5,150)}S500\r\n'))
              self.ball_save_dict.pop(key)
              self.wait_rotation()
              
              break
          
          
        def catch_ball_at_x_y_z(x,y,z):
          print('to go',x,y,z)
          ang = solve_angle(x,y,z)
          speed = '500'
          cmd_p0 = f'#0P{self.deg2period(0,ang[0])}S{speed}#1P{self.deg2period(1,ang[1])}S{speed}#2P{self.deg2period(2,ang[2])}S{speed}#3P{self.deg2period(3,ang[3])}S{speed}#5P1550S{speed}\r\n'
          print('cal',get_hand(ang[0],ang[1],ang[2],ang[3]))
          self.serial.write(str.encode(cmd_p0)) #先不移動0號陀機，以防撞到木板
          time.sleep(7)
          ang = solve_angle(x,y,z)
          cmd_p0 = f'#0P{self.deg2period(0,ang[0])}S{speed}#1P{self.deg2period(1,ang[1])}S{speed}#2P{self.deg2period(2,ang[2])}S{speed}#3P{self.deg2period(3,ang[3])}S{speed}\r\n'
          print(cmd_p0)
          self.serial.write(str.encode(cmd_p0)) #先不移動0號陀機，以防撞到木板
          print('cal',get_hand(ang[0],ang[1],ang[2],ang[3]))
          time.sleep(5)
          self.serial.write(str.encode('#5P1300S500T1000\r\n')) #移動0號陀機
          #time.sleep(3)
          #self.serial.write(str.encode('#0P680S500T3000\r\n')) #移動0號陀機
          

          """
          ang = solve_angle(x,y,z)#solve_angle(x+12*x//411,y+27*y//400,z)
          if ang[0]-self.pre_angle[0] > 0:
              print('------------------------')
              cmd_p0 = f'#0P680S500#1P{int(-ang[1]*1000//90+1500)}S500#2P{int(-ang[2]*1000//90+1500)}S500#5P1550S500T8000\r\n'
              self.serial.write(str.encode(cmd_p0)) #先不移動0號陀機，以防撞到木板
              time.sleep(10)
              cmd_p1 = f'#3P{int(-ang[3]*1000//90+1500)}S500\r\n'
              self.serial.write(str.encode(cmd_p1))
              time.sleep(10)
              cmd_p2 = f'#0P{int(-ang[0]*1000//90+1500)}S500T8000\r\n'
              self.serial.write(str.encode(cmd_p2)) #移動0號陀機
              time.sleep(10)
              self.serial.write(str.encode('#5P1300S500T1000\r\n')) #移動0號陀機
              time.sleep(10)
              self.serial.write(str.encode('#0P680S500T3000\r\n')) #移動0號陀機
          else:
              cmd_p1 = f'#1P{int(-ang[1]*1000//90+1500)}S500#2P{int(-ang[2]*1000//90+1500)}S500#3P{int(-ang[3]*1000//90+1500)}S500T2000\r\n'
              self.serial.write(str.encode(cmd_p1)) #先不移動0號陀機，以防撞到木板
              cmd_p2 = f'#0P{int(-ang[0]*1000//90+1500)}S500T5000\r\n'
              self.serial.write(str.encode(cmd_p2)) #移動0號陀機
              time.sleep(7)
              self.serial.write(str.encode('#5P1300S500T1000\r\n')) #移動0號陀機
              time.sleep(5)
              self.serial.write(str.encode('#0P1200S500T2000\r\n')) #移動0號陀機
          """
          #self.serial.write(str.encode(f'#0#5P1200S500T500\r\n')) #抓取物體
          
          self.pre_angle = [*ang,0,0]
          self.pre_angle[0] = 27
        if len(self.ball_save_dict) != 0:
            #worker = Thread(target=catch_ball_at_x_y_z, args=(218,122,16))
            #worker = Thread(target=catch_ball_at_x_y_z, args=(ball[0][0],ball[0][1],ball[0][2]))
            worker = Thread(target=dynamic_catch_ball_at_x_y_z)
            worker.setDaemon(True)
            worker.start()
    def set_image(self):
        def rotate(image, angle, center=None, scale=1.0):
            (h, w) = image.shape[:2]
            if center is None:
                center = (w / 2, h / 2)
            M = cv2.getRotationMatrix2D(center, angle, scale)
            rotated = cv2.warpAffine(image, M, (w, h))
            return rotated
        def cal_obj_side(img):
            try:
                s0 = time.time()
                h0,w0,_ = img.shape
                #l_nw = np.array([0,0,0])
                #u_nw = np.array([80,255,60])
                #img_nw = cv2.inRange(img,l_nw,u_nw)
                
                img_copy = img.copy()
                #img = cv2.bitwise_and(img,img,mask=img_nw)
                hls =  cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
                hls_mask = cv2.inRange(hls, np.array([60,0,100]),np.array([100,60,255]) ) #將綠色濾除
                hls_mask = cv2.medianBlur(hls_mask,7)
                
                img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                #mean_color = int(cv2.mean(img_gray,img_nw)[0] //15 *15)
                #l_nw_y = np.array([30,50,60])
                #u_nw_y = np.array([185,220,250])

                #img_nw_y = cv2.inRange(img_copy,l_nw_y,u_nw_y)
                #img_copy = cv2.bitwise_and(img_copy,img_copy,mask=img_nw_y)
                """
                hls_copy =  cv2.cvtColor(img_copy, cv2.COLOR_BGR2HLS)
                hls_mask_yellow = cv2.inRange(hls_copy, np.array([5,30,40]),np.array([50,230,255]) )
                """
                start_y = h0//10
                contours,_ = cv2.findContours(hls_mask[start_y:h0-1,:],cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
                if not self.lock_board_flag:
                  min_x, min_y,max_x, max_y = self.get_board(contours,w0,h0,start_y)
                  if max_y == 0 or max_x ==0:
                      max_x = w0
                      min_x = 0
                else:
                  min_x, min_y,max_x, max_y = [self.side_min_x,self.side_min_y,self.side_max_x,self.side_max_y]
                """
                if max_y != 0 and max_x !=0:
                    roiImg = hls_mask_yellow[0:max_y,min_x:max_x]
                else:
                    roiImg = hls_mask_yellow
                    max_x = w0
                    min_x = 0
                #roiImg = cv2.medianBlur(roiImg, 11)
                #self.side_cuda.upload(roiImg)
                #self.side_blur.apply(self.side_cuda,self.side_cuda)
                #self.side_cuda.download(roiImg)
                
                roiImg = cv2.medianBlur(roiImg, 7)
                contours_p,_ = cv2.findContours(roiImg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                """
                rect_h = max_y-min_y
                ratio_y = 25/rect_h
                ratio_x = 411/(max_x-min_x)
                border_w = (max_x-min_x)
                roi_height = max_y
                circles,ball_side = detector2.get_side_ball(img[0:max_y,min_x:max_x],min_x,0,min_y,ratio_x,ratio_y,border_w,max_y) 
                self.ball_side = ball_side
                #circles = self.get_side_circle(contours_p,min_x,max_x,0,min_y,ratio_x,ratio_y,h0-max_y,border_w,roiImg)
                s2 = time.time()
                #print(s1-s0,s2-s1)
                self.side_data = [[min_x,min_y,max_x,max_y,circles,hls_mask]]
            except Exception as e:
                string = "error"
                print(e)

        def cal_obj_top(img):
            try:
                h0,w0,_ = img.shape 
                center_x = w0//2
                center_y = h0//2
                if not self.lock_board_flag:
                  hls =  cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
                  hls_mask = cv2.inRange(hls, np.array([75,40,60]),np.array([100,220,255]) ) #將綠色濾除
                  contours,_ = cv2.findContours(hls_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                  min_x, min_y,max_x, max_y = self.get_roi(contours,w0,h0)
                  """
                  max_white = 0
                  box_save = []
                  img_copy = img[min_y+30:max_y-30,min_x+80:max_x-30].copy()
                  
                  hls_mask_new = cv2.inRange(img_copy, (150,150,130),(255,255,255))
                  hls_mask_new = cv2.medianBlur(hls_mask_new,7)
                  contours_hand,_ = cv2.findContours(hls_mask_new,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                  points = []
                  for c in contours_hand:
                    hull = cv2.convexHull(c)
                    points.append(hull)
                    x, y, w, h = cv2.boundingRect(c)
                    non_zero = cv2.countNonZero(hls_mask_new[y:y+h,x:x+w])
                    white = (non_zero)/(w*h)
                    print('white',white)
                    #cv2.rectangle(img_copy, (x,y), (x+w,y+h), (255,0,255), 5)
                    if max_white<white and white<0.9:
                      
                  
                      max_white = white
                      rect = cv2.minAreaRect(c) # 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
                      box = cv2.boxPoints(rect) # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
                      box = np.int0(box)
                      box_save = box                    
                  print('---------------')
                  #cv2.rectangle(img_copy, (x,y), (x+w,y+h), (255,0,255), 5)
                  #img_copy = cv2.drawContours(img_copy, points, -1, (255, 0, 255), -1)

                  

                  if box_save != []:
                    print(box_save)
                  img_copy = cv2.drawContours(img_copy,contours_hand,-1,(255,0,255),-1)
                  cv2.imshow('top_mask',hls_mask)
                  cv2.waitKey(1)
                  """
                else:
                  min_x, min_y,max_x, max_y = [self.board_min_x,self.board_min_y,self.board_max_x,self.board_max_y]
                  hls_mask = img
                rect_h = max_y-min_y
                rect_w = max_x-min_x
                ratio = 400/rect_h
                """
                if max_x != 0:
                    roiImg = cv2.bitwise_not(hls_mask)[min_y:max_y,min_x:max_x]
                    contours,_ = cv2.findContours(roiImg,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                else:
                    roiImg = []
                """
                self.conveyor_area = [(int(min_x+rect_w*2/3),int(min_x+rect_w)),(min_y,max_y)] # [[x_min,x_max],[y_min,y_max]]
                circles = detector1.get_top_ball(img[min_y:max_y,min_x:max_x],min_x,min_y,ratio,rect_w,rect_h,self.conveyor_area,center_x,center_y) 
                #print(detector.get_ball(img))
                #circles = self.get_top_circle(contours,min_x,max_x,min_y,ratio,rect_h,roiImg)
                #print(rect_w*ratio)
                self.top_data = [[min_x,min_y,max_x,max_y,circles,rect_w*ratio,ratio]]
                s4 = time.time()
                #print('part1',s2-s1,'part2',s3-s2,'part3',s4-s3)
            except Exception as e:
                string = 'hello'
                cl, exc, tb = sys.exc_info() #取得Call Stack
                lastCallStack = traceback.extract_tb(tb)[-1]
                print('top',e.args[0],lastCallStack)
        def process_top():
            try:
                _, img = self.top_capture.read()
                #img = rotate(img,180)
                img_cp = img.copy()
                tmp = int(self.width*0.9)
                line_w = tmp//100
                if self.top_count % 1 == 0:
                  cal_obj_top(img)
                  #worker = Thread(target=cal_obj_top, args=(img,))
                  #worker.setDaemon(True)
                  #worker.start()
                #
                if self.top_data!=[None]:
                  top_data = self.top_data[0]
                  if self.lock_board_flag and self.board_max_y==0:
                    self.board_min_x = top_data[0]
                    self.board_min_y = top_data[1]
                    self.board_max_x = top_data[2]
                    self.board_max_y = top_data[3]
                  ratio = top_data[6]
                  cv2.rectangle(img_cp, (top_data[0], top_data[1]), (top_data[2], top_data[3]), (255,0,255), line_w//5)
                  if self.conveyor_area[0] != None:
                    cv2.rectangle(img_cp, (self.conveyor_area[0][0],self.conveyor_area[1][0]), (self.conveyor_area[0][1],self.conveyor_area[1][1]), (0,255,255), line_w//5)
                  #cv2.imshow('hls_mask',top_data[6])
                  #cv2.waitKey(1)
                  ball_top = []
                  all_dis = {}
                  ball_nums = {}
                  tmp_ball_nums = {}
                  for i0 in self.ball_save_dict:
                    ball_save = self.ball_save_dict[i0]
                    all_dis[i0] = []
                    b_x,b_y,_,_,_,_,_,_,_ = ball_save
                    for i in range(len(top_data[4])):
                      circle = top_data[4][i]
                      x,y = circle[0],circle[1]
                      distance = ((b_x-x)**2+(b_y-y)**2)**(1/2)
                      all_dis[i0] += [distance]
                  for key in all_dis:
                    if len(all_dis[key])==0:
                      continue
                    min_d = min(all_dis[key])
                    min_index = all_dis[key].index(min_d)
                    if not tmp_ball_nums.get(min_index):
                      tmp_ball_nums[min_index] = {'d': [],'key': []}
                    tmp_ball_nums[min_index]['d'] += [min_d]
                    tmp_ball_nums[min_index]['key'] += [key]
                  for key in tmp_ball_nums:
                    all_d = tmp_ball_nums[key]['d']
                    all_key = tmp_ball_nums[key]['key']
                    ball_nums[key] = all_key[all_d.index(min(all_d))]
                  for i0 in list(self.ball_save_dict.keys()):
                    point = self.ball_save_dict[i0]
                    if i0 not in list(ball_nums.values()):
                      board_w = top_data[5]
                      board_h = top_data[3]-top_data[1]
                      #print(board_w)
                      x = point[0]
                      y = point[1] + (1-detector1.center_y_ratio)*board_h
                      f1 = abs(x-0)/board_w<=0.01
                      f2 = abs(board_w-x)/board_w<=0.01
                      f3 = abs(y-0)/board_h<=0.01
                      f4 = abs(board_h-y)/board_h<=0.01
                      #print((x-0)/board_w,(board_w-x)/board_w,(y-0)/board_h,(board_h-y)/board_h)
                      if f1 or f2 or f3 or f4:
                        print('remove',f1,f2,f3,f4,self.ball_save_dict.pop(i0))
                  now_time = time.time()
                  time_diff = now_time - self.pre_time
                  self.pre_time = now_time
                  not_showing_num = list(self.ball_save_dict.keys())
                  for i in range(len(top_data[4])):
                    circle = top_data[4][i]
                    if i not in ball_nums:
                      try:
                        now_num = max(self.ball_save_dict.keys()) +1
                      except Exception as e:
                        print(e)
                        now_num = 0
                      ball_nums[i] = now_num
                      v = None
                      a = None
                    else:
                      now_num = ball_nums[i]
                      v = [int((circle[0]-self.ball_save_dict[now_num][0])/time_diff),int((circle[1]-self.ball_save_dict[now_num][1])/time_diff)]
                      if self.ball_save_dict[now_num][4]:
                        a = [int((v[0]-self.ball_save_dict[now_num][4][0])/time_diff),int((v[1]-self.ball_save_dict[now_num][4][1])/time_diff)]
                      else:
                        a = None
                    if now_num in not_showing_num:
                      not_showing_num.remove(now_num)
                    self.ball_save_dict[now_num] = [circle[0],circle[1],circle[5],circle[4],v,a,circle[2],circle[3],ratio]
                    distances = []
                    cv2.circle(img_cp,circle[2],circle[3], (255, 0, 255), line_w)
                    cv2.putText(img_cp, f'({circle[0]},{circle[1]},{circle[5]}) D={circle[4]}', (circle[2][0]-120,circle[2][1]), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 0, 0), 1, cv2.LINE_AA)
                    cv2.putText(img_cp, f'v={v}, a={a}', (circle[2][0]-120,circle[2][1]+50), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 0, 0), 1, cv2.LINE_AA)
                    cv2.putText(img_cp, f'{now_num}', (circle[2][0]-circle[3]//2,circle[2][1]), cv2.FONT_HERSHEY_SIMPLEX,line_w//2, (0, 0, 0), 1, cv2.LINE_AA)
                    ball_top += [[circle[0],circle[1],circle[4]]]
                  self.ball_top = sorted(ball_top)
                  height, width,_ = img_cp.shape
                  """
                  for i in not_showing_num:
                    x,y,z,D,v,a,image_cord,image_R,ratio = self.ball_save_dict[i]
                    if v!=None:
                      
                      #if a!=None:
                      #  v[0] = v[0] + a[0]*time_diff
                      #  v[1] = v[1] + a[1]*time_diff
                      
                      image_cord_new = (int(image_cord[0]+v[0]/ratio*time_diff),int(image_cord[1]+v[1]/ratio*time_diff))
                    else:
                      v = [0,0]
                      image_cord_new = image_cord
                    
                    self.ball_save_dict[i] = [int(x+v[0]),int(y+v[1]),z,D,v,a,image_cord_new,image_R,ratio]
                    if width-1>=image_cord_new[0]>=0 and height-1>=image_cord_new[1]>=0:
                      cv2.circle(img_cp,image_cord_new,image_R, (0, 255, 0), line_w)
                      cv2.putText(img_cp, f'({x},{y},{z}) D={D}', (image_cord_new[0]-120,image_cord_new[1]), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 255, 0), 1, cv2.LINE_AA)
                      cv2.putText(img_cp, f'v={v}, a={a}', (image_cord_new[0]-120,image_cord_new[1]+50), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 255, 0), 1, cv2.LINE_AA)
                      cv2.putText(img_cp, f'{i}', (image_cord_new[0]-image_R//2,image_cord_new[1]), cv2.FONT_HERSHEY_SIMPLEX,line_w//2, (0, 255, 0), 1, cv2.LINE_AA)
                  """
                self.top_count += 1
                #
                """
                mean_color = int(min(cv2.mean(img_cp)[:-1]))
                Lower1 = np.array([int(mean_color//5), int(mean_color//5), 0]) #顏色下限
                Upper1 = np.array([ int(mean_color*2.5), int(mean_color*5), mean_color//5]) #顏色上限
                img1 = cv2.inRange(img_cp, Lower1, Upper1) #將綠色濾除
                img1 = cv2.blur(img1, (3,3))
                img1 = cv2.resize(img1, (tmp,tmp//2), interpolation = cv2.INTER_AREA)
                cv2.imshow('img_top',img1)
                cv2.waitKey(1)
                """
                #

                img_cp = cv2.cvtColor(img_cp, cv2.COLOR_BGR2RGB)
                img_cp = cv2.resize(img_cp, (tmp,tmp//2), interpolation = cv2.INTER_AREA)
                height, width,_ = img_cp.shape
                bytesPerLine = 3 * width
                qImg = QImage(img_cp.data, width, height, bytesPerLine, QImage.Format_RGB888)
                pixmap = QPixmap(qImg)
                self.top_image.setPixmap(pixmap)
                self.top_image.show()
            except Exception as e:
                string = 'hello'
                cl, exc, tb = sys.exc_info() #取得Call Stack
                lastCallStack = traceback.extract_tb(tb)[-1]
                print('top_img',e.args[0],lastCallStack)
        def process_side():
            try:
                _, img = self.side_capture.read()
                #img = rotate(img,180)
                img_cp = img.copy()
                tmp = int(self.width*0.9)
                line_w = tmp//100
                
                #hsv = cv2.cvtColor(img_cp, cv2.COLOR_BGR2HSV)
                if self.side_data!=[None]:
                    side_data = self.side_data[0]
                    if self.lock_board_flag and self.side_max_y==0:
                      self.side_min_x = side_data[0]
                      self.side_min_y = side_data[1]
                      self.side_max_x = side_data[2]
                      self.side_max_y = side_data[3]
                    cv2.rectangle(img_cp, (side_data[0], side_data[1]), (side_data[2], side_data[3]), (255,0,255), line_w//5)
                    ball_left = []
                    for circle in side_data[4]:
                        cv2.circle(img_cp,circle[2],circle[3], (255, 0, 255), line_w//5)
                        cv2.putText(img_cp, f'({circle[0]},{circle[1]})', circle[2], cv2.FONT_HERSHEY_SIMPLEX,line_w//5, (255, 0, 255), 2, cv2.LINE_AA)
                        ball_left += [[circle[0],circle[1]]]
                    self.ball_left = sorted(ball_left)
                    cv2.imshow('purple',side_data[5])
                    cv2.waitKey(100)
                    #cv2.imshow('hsv',hsv)
                    #cv2.waitKey(100)
                if self.side_count % 1 == 0:
                  cal_obj_side(img)
                  #worker = Thread(target=cal_obj_side, args=(img,))
                  #worker.setDaemon(True)
                  #worker.start()

                self.side_count += 1
                #
                """
                mean_color = int(min(cv2.mean(img_cp)[:-1]))
                Lower1 = np.array([int(mean_color/10*7*2), int(mean_color/10*8*2), int(mean_color*2.3*0.8)]) #顏色下限
                Upper1 = np.array([ int(mean_color*1.1*3), int(mean_color*1.4*3), int(mean_color*1.6*3)]) #顏色上限
                img1 = cv2.inRange(img_cp, Lower1, Upper1) #將綠色濾除
                img1 = cv2.medianBlur(img1, 11)
                img1 = cv2.resize(img1, (tmp,tmp//2), interpolation = cv2.INTER_AREA)
                cv2.imshow('img',img1)
                cv2.waitKey(1)
                """
                #

                img_cp = cv2.cvtColor(img_cp, cv2.COLOR_BGR2RGB)
                img_cp = cv2.resize(img_cp, (tmp,tmp//2), interpolation = cv2.INTER_AREA)
                height, width,_ = img_cp.shape
                bytesPerLine = 3 * width
                qImg = QImage(img_cp.data, width, height, bytesPerLine, QImage.Format_RGB888)
                pixmap = QPixmap(qImg)
                self.side_image.setPixmap(pixmap)
                self.side_image.show()
            except Exception as e:
                string = 'hello'
                cl, exc, tb = sys.exc_info() #取得Call Stack
                lastCallStack = traceback.extract_tb(tb)[-1]
                print('side_img',e.args[0],lastCallStack)
        process_top()

        #process_side()
        #self.ball_side = [[123,12]]
        #self.side_data=[]
        """
        if self.top_data != [None]:
            ball_top = self.ball_top[0]
            self.ball = [[ball_top[0],ball_top[1],16,ball_top[2]]]
            #self.ball = [[ball_top[0],ball_top[1],16,ball_top[2]] for ball_top,ball_side in zip(self.ball_top,self.ball_side)]
            #self.ball = [[ball_top[0],ball_top[1],ball_side[1],ball_top[2]] for ball_top,ball_side in zip(self.ball_top,self.ball_side)]
            #print(self.ball)
        """
    def deg2period(self,num,ang):
      """
      if num != 3 and num != 0:
        return int((self.max_pwm[num]-self.min_pwm[num])/(self.max_ang[num]-self.min_ang[num])*(ang-self.min_ang[num])+self.min_pwm[num])
      elif num ==0:
        return int(np.polyval(self.val0,ang))
      else:
        return int(np.polyval(self.val3,ang+self.prefix_deg3))
      """
      return int(np.polyval(self.val[num],ang))
    def period2deg(self,num,period):
      def get_deg3(x,*args, **kwargs):
        period = args[0]
        return [np.polyval(self.val3,x+self.prefix_deg3)[0]-period]
      def get_deg0(x,*args, **kwargs):
        period = args[0]
        return [np.polyval(self.val0,x)[0]-period]
      def get_deg(x,*args, **kwargs):
        period,num = args
        return [np.polyval(self.val[num],x)[0]-period]

      """
      if num != 3 and num != 0:
        return (self.max_ang[num]-self.min_ang[num])/(self.max_pwm[num]-self.min_pwm[num])*(period-self.min_pwm[num])+self.min_ang[num]
      elif num ==0:
        ang0 = least_squares(fun=get_deg0,loss='soft_l1',tr_solver='exact',x0=0,args=(period,))
        print('period',period,np.polyval(self.val0,ang0.x[0]))
        return ang0.x[0]
      else:
        ang3 = least_squares(fun=get_deg3,loss='soft_l1',tr_solver='exact',x0=0,args=(period,))
        print('period',period,np.polyval(self.val3,ang3.x[0]+self.prefix_deg3))
        return ang3.x[0]
      """
      ang = least_squares(fun=get_deg,loss='soft_l1',tr_solver='exact',x0=0,args=(period,num))
      #print('period',period,np.polyval(self.val[num],ang.x[0]))
      return ang.x[0]
    def wait_rotation(self):
      while 1:
        self.serial.write(str.encode('Q\r\n'))
        res = self.serial.read()
        if res==b'.':
          break
    def get_pre_angle(self):
      self.wait_rotation()
      #self.pre_angle = [0,0,0,0]
      self.pre_angle = []
      for i in range(4):
        self.serial.write(str.encode(f"QP{i}\r\n"))
        try:
          res = list(self.serial.read())[0]*10
        except:
          res = 1500
        self.pre_angle += [self.period2deg(i,res)]

      print('pre',self.pre_angle)
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.threadpool = QThreadPool()
        self.threads = []

        self.val3 = [7.118088694474786e-34,-1.3623676833915108e-30,6.679657776440796e-28,-1.0348724692233468e-25,-8.444614556302993e-24,3.803543911131992e-21,-1.416484420729551e-19,-4.705344947091237e-17,3.3937926815883385e-15,2.880851562274015e-13,-2.49804049132929e-11,-9.832223770834514e-10,7.445876814208426e-08,1.7793671135347453e-06,-3.8016616171468486e-05,-0.0016213018755782587,7.018496032336845,1209.9563238491194] #[-6.67530754,1710.80280874] #[-1.58322643e-08,4.43845777e-07,5.11729574e-04,1.01752766e-02,-9.61043023e+00,1.71098717e+03]
        #self.val0 = [4.639475565634133e-09,3.766000024436108e-07,-0.0001536824266673348,-0.003263913532938517,11.897078491998647,1437.0] #[1.2567152902115243e-28,-1.6531428810099285e-26,-3.6594451344026426e-24,5.081597673408913e-22,4.24263756497142e-20,-6.232527917884899e-18,-2.509683520329688e-16,3.82222015964742e-14,8.33638341394498e-13,-1.1725038887293647e-10,-1.8178702733392338e-09,1.3697869700368183e-07,3.4862736455458e-06,7.376841477013416e-05,-0.005128159084342743,-0.20295959935806704,12.619342277472743,1428.3096843643798]
        self.val0 = [4.639475565634133e-09,3.766000024436108e-07,-0.0001536824266673348,-0.003263913532938517,11.897078491998647,1437.0]
        self.val1 = [-7.796712845197698e-09,3.9093904883050645e-07,0.00016081870288337554,-0.0053505967875676725,-11.853410341491886,1372.0] #[1.2567152902115243e-28,-1.6531428810099285e-26,-3.6594451344026426e-24,5.081597673408913e-22,4.24263756497142e-20,-6.232527917884899e-18,-2.509683520329688e-16,3.82222015964742e-14,8.33638341394498e-13,-1.1725038887293647e-10,-1.8178702733392338e-09,1.3697869700368183e-07,3.4862736455458e-06,7.376841477013416e-05,-0.005128159084342743,-0.20295959935806704,12.619342277472743,1428.3096843643798]
        self.val2 = [2.0939183469629637e-08,1.9178225920151715e-06,-0.0003414550075445629,-0.022519595668021166,12.604751079774868,1549.0]
        self.val5 = [-7.648453968480813e-12,4.8895274300084295e-09,-1.2197274482963686e-06,0.0001479862788630983,-0.008903614091829406,0.24814643212601334,8.463614220989971,560.3525020096391]
        self.val = [self.val0,self.val1,self.val2,self.val3,[],self.val5]
        self.prefix_deg3 = 0
        print("Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())
        self.counter = 0
        layout = QVBoxLayout()
        self.top_data = [None]
        self.side_data = [None]
        self.ball_top = []
        self.ball_side = []
        self.ball = []
        self.ball_save_dict = {}
        self.eeg_label = QLabel("EEG: unread")
        self.eeg_label.setAlignment(Qt.AlignRight)
        self.eeg_label.setFont(QFont('Times',24))
        self.domain1 = 'http://192.168.43.64:4747'
        self.domain2 = 'http://192.168.137.227:4747'
        self.top_image = QLabel()
        self.side_image = QLabel()
        self.board_min_x = 0
        self.board_min_y = 0
        self.board_max_x = 0
        self.board_max_y = 0
        self.stop_flag = True
        self.side_min_x = 0
        self.side_min_y = 0
        self.side_max_x = 0
        self.side_max_y = 0
        deg_500 = self.period2deg(3,500)
        deg_2500 = self.period2deg(3,2500)
        self.conveyor_area = [None]
        """
        self.p = [[[121,-86],[2500,500]],    #1437 -> 0度 615(-90度)~2500(89.5度)
                  [[80,-50.04],[500,1867]],   #1372   500(80度)~1341(0度)
                  [[82.7,-87.3],[2500,525]],#12#4.41    #1549  2500(85度)~525(-90度) #0deg: 1573
                  [[deg_500,deg_2500],[500,2500]]]   #1427  #523(90度)~2500(-87度)
        """
        self.p = []
        for i in range(4):
          tmp = [[self.period2deg(i,pwm),pwm] for pwm in [500,2500]]
          tmp.sort(reverse=True)
          self.p += [[[tmp[0][0],tmp[1][0]],[tmp[0][1],tmp[1][1]]]]
        print(self.p)
        #555 -90deg,1840 90deg,1198 0deg,2500 175deg
        ang,pwm_period = list(zip(*self.p))
        max_ang,min_ang = list(zip(*ang))
        max_pwm,min_pwm = list(zip(*pwm_period))
        self.max_pwm = max_pwm
        self.min_pwm = min_pwm
        self.max_ang = max_ang
        self.min_ang = min_ang
        print(max_pwm,min_pwm)
        print(max_ang,min_ang)
        """
        self.top_cuda = cv2.cuda_GpuMat()
        self.top_cuda.create((1920,1080),cv2.CV_8UC1)
        self.side_cuda = cv2.cuda_GpuMat()
        self.side_cuda.create((1920,1080),cv2.CV_8UC1)
        self.top_blur = cv2.cuda.createMedianFilter(cv2.CV_8UC1,11,11)
        self.side_blur = cv2.cuda.createMedianFilter(cv2.CV_8UC1,121)
        """
        self.top_count = 0
        self.side_count = 0
        url1 = f'{self.domain1}/video'
        url2 = f'{self.domain2}/video'
        ports = serial.tools.list_ports.comports()
        #print("manufacturer",ports[0].name)
        arduino_exist_flag = False
        eeg_exist_flag = False
        self.serial = None
        if len(ports) != 0:
          for p in ports:
            serial_tp = serial.Serial(p[0], 115200, timeout=2)
            if p.name=='rfcomm0' or p.name=='COM3':
              self.serial_eeg = serial_tp
              eeg_exist_flag = True
              print('EEG here')
            else:
              read_str = serial_tp.read(4)
              print(read_str)
              read_str = read_str.decode()
              print('read_str',read_str)
              if read_str not in ['init','STOP','STAR']:
                print('servo controller')
                self.serial = serial_tp
                self.initial_hand()
              else:
                arduino_exist_flag = True
                self.serial_arduino = serial_tp
                print('arduino is',p)
        self.width = self.frameGeometry().width()
        self.height = self.frameGeometry().height()
        if not self.serial:
          self.pre_angle = [0,0,0,0]
        else:
          worker = Thread(target=self.get_pre_angle)
          worker.setDaemon(True)
          worker.start()
        

        #self.top_capture = cv2.VideoCapture('top_0603.mp4')#
        #self.top_capture = cv2.VideoCapture(url1)
        self.top_capture = cv2.VideoCapture(0)#'/dev/video4', cv2.CAP_V4L2)
        #self.top_capture = cv2.VideoCapture(url2)
        #self.side_capture = cv2.VideoCapture('side_0603.mp4')#cv2.VideoCapture(url)
        #self.side_capture = cv2.VideoCapture(2)#'/dev/video5', cv2.CAP_V4L2)
        #self.side_capture = cv2.VideoCapture(url2)
        self.lock_board_flag = False
        self.set_image()
        b = QPushButton("catch ball")
        b.pressed.connect(self.catch_ball)
        c = QPushButton("lock board")
        c.pressed.connect(self.lock_board)
        d = QPushButton("init hand")
        d.pressed.connect(lambda: self.initial_hand(True))

        e = QPushButton("stop catch")
        e.pressed.connect(self.stop_catch)

        layout.addWidget(self.eeg_label)
        layout.addWidget(self.top_image)
        layout.addWidget(self.side_image)
        layout2 = QHBoxLayout()
        layout2.addWidget(b)
        layout2.addWidget(c)
        layout.addLayout(layout2)
        layout.addWidget(d)
        layout.addWidget(e)
        w = QWidget()
        w.setLayout(layout)
        self.arduino_output = ''
        self.setCentralWidget(w)
    
        self.show()
        """
        a=85 #89
        b=105
        c=148
        d=172
        bending_height = 0 #單位:mm

        small_r = 20
        d2r = math.pi/180
        def get_hand(t0,t1,t2,t3):
          ang0 = -t0
          ang1 = t1
          ang2 = t2
          ang3 = t3
          alpha = ang0 * d2r
          beta = (-ang0+ang1) * d2r
          gamma = (-ang0+ang1+ang2) *d2r
          delta = ang3 * d2r
          #print(alpha,beta,gamma,delta)
          z=(a+b*math.cos(alpha)+c*math.sin(beta)+d*math.sin(gamma)) - bending_height
          r=b*math.sin(alpha)+c*math.cos(beta)+d*math.cos(gamma)
          #print(r)
          x=r*math.cos(delta) + small_r*math.sin(delta)
          y=r*math.sin(delta) - small_r*math.cos(delta)
          return [x,y,z]
        
        print(get_hand(0,0,90,0))
        """
        self.pre_time = time.time()
        self.timer = QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.recurring_timer)
        self.timer.start()
        if arduino_exist_flag:
          self.timer_arduino = QTimer()
          self.timer_arduino.setInterval(100)
          self.timer_arduino.timeout.connect(self.arduino_timer)
          self.timer_arduino.start()
        if eeg_exist_flag:
          """
          self.timer_eeg = QTimer()
          self.timer_eeg.setInterval(700)
          self.timer_eeg.timeout.connect(self.eeg_timer)
          self.timer_eeg.start()
          """
          worker = Thread(target=self.eeg_timer)
          worker.setDaemon(True)
          worker.start()
    def oh_no(self):
        url_led = f'{self.domain}/cam/1/led_toggle'
        cap123 = cv2.VideoCapture(url_led)
        cap123.read()
    def get_ball_for_arduino(self):
      return '/'.join([f'{round(ball[0]/10)*10},{round(ball[1]/10)*10},{round(ball[2]/10/2)*10},{round(ball[2]/5)*5}' for ball in self.ball_top])
    def recurring_timer(self):
        self.set_image()
    def arduino_timer(self):
      s1 = time.time()
      print('rotate start')
      self.serial_arduino.read()
      tmp = self.get_ball_for_arduino()
      if tmp != self.arduino_output:
        self.arduino_output = tmp
        print(tmp)
        self.serial_arduino.write(str.encode(f'{tmp}\n'))
      print('rotate finish',time.time()-s1)
    def stop_catch(self):
      self.stop_flag = True
    def eeg_timer(self):
      count = 0
      big_data = [None] * 35
      check_sum = 0
      while 1:
        r_list = list(self.serial_eeg.read(512))
        for r in r_list:
          flag = ((count==0 and r==0xAA) 
                  or (count==1 and r==0xAA) 
                  or (count==2 and r==0x20) 
                  or (count==3 and r==0x02) 
                  or (count==4 and r==0) 
                  or (count==5 and r==0x83)
                  or (count==6 and r==0x18) 
                  or (count >=7 and count < 35))
          if flag:       #同步信号 
            big_data[count] = r
            if 3<=count:
              check_sum += r
            count += 1
          elif count==35:
            check_sum = (~check_sum)&0xff
            print(check_sum,r)
            if check_sum==r:
              signalquality = big_data[4]
              attention = big_data[32]
              meditation = big_data[34]
              self.eeg_label.setText(f'EEG: {meditation}')
              if int(meditation)<40:
                self.stop_flag = True
              print('signalquality',signalquality,'attention',attention,'meditation',meditation)
            else:
              print('invalid')
            count = 0
            check_sum = 0
app = QApplication([])
window = MainWindow()
app.exec_()