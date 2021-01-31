import cv2
import numpy as np
import math
import serial
import serial.tools.list_ports
import time
import yaml
def rotate(image, angle, center=None, scale=1.0):
    (h, w) = image.shape[:2]
    if center is None:
        center = (w / 2, h / 2)
    M = cv2.getRotationMatrix2D(center, angle, scale)
    rotated = cv2.warpAffine(image, M, (w, h))
    return rotated
def wait_rotation(s):
	while 1:
		s.write(str.encode('Q\r\n'))
		res = s.read()
		if res==b'.':
		  break
fname = 'Photo-1.jpeg'
cap = cv2.VideoCapture(0)
ports = serial.tools.list_ports.comports()
if len(ports) != 0:
  s = serial.Serial(ports[-1][0], 115200, timeout=5)

pos_flag = True
i3 = 1200 
diff = 10
xs = []
ys = []
while(True):
  change = diff if pos_flag else -diff
  i3 += change
  if i3>2500 or i3<1200:
    pos_flag = not pos_flag
    change = diff if pos_flag else -diff
    i3 += (change*2)


  # 從攝影機擷取一張影像
  ret, img_ = cap.read()
  #img_ = img_[180:230,280:350]
  img_ = img_[100:340,250:550]#[220:280,400:450]
  #img_ = rotate(img_,90)
  h,w,_ = img_.shape
  img_ = cv2.resize(img_,(w,h))
  img = cv2.cvtColor(img_,cv2.COLOR_BGR2GRAY)
  img1 = img_.copy()
  
  #sz = (300,300)
  #img = cv2.imread(fname,0)[100:400,200:400]
  #img = cv2.resize(img,sz)
  #img_ = cv2.imread(fname)[100:400,200:400]
  
  #img_ = cv2.resize(img_,sz)
  thresh = cv2.inRange(img_, (190,190,190),(255,255,255))
  thresh = cv2.medianBlur(thresh,3)
  #thresh = cv2.bitwise_not(thresh)
  edges = cv2.Canny(thresh, 0,255)
  
  cnts,_=cv2.findContours(edges,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  max_area = -1
  max_cnt = cnts[-1]
  for c in cnts:
    x0,y0,w0,h0 = cv2.boundingRect(c)
    area = w0*h0
    if max_area<area:
      max_area = area
      max_cnt = c
  hull = cv2.convexHull(max_cnt)
  rect = cv2.minAreaRect(hull) # 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
  box = cv2.boxPoints(rect) # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
  box = np.int0(box)
  w1,h1 = rect[1]
  if w1>h1:
    ang = rect[-1]
  else:
    ang = -(90+rect[-1])
  if ang!=0 and ang!=90:
    xs += [i3]
    ys += [ang]
    print(f'i3:{i3}, ang={ang}, w={w1}, h={h1}')
  img1 = cv2.drawContours(img1, [box], 0, (255, 0, 255), 1)
  

  cv2.imshow('img',edges)
  cv2.imshow('out1',thresh)
  cv2.imshow('img1',cv2.resize(img1,(w*4,h*4)))
  #cv2.imshow('img1',img1)
  key = cv2.waitKey(20)
  if key==ord('q'):
    break