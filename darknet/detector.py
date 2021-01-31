import darknet
import cv2
import random
import glob
import numpy as np
def image_detection(image_path, network, class_names, class_colors, thresh):
    # Darknet doesn't accept numpy images.
    # Create one with image we reuse for each detect
    darknet_image = darknet.make_image(resize_width, resize_height, 3)

    image = cv2.imread(image_path)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image_rgb, (resize_width, resize_height),
                               interpolation=cv2.INTER_LINEAR)

    darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
    detections = darknet.detect_image(network, class_names, darknet_image, thresh=thresh)
    darknet.free_image(darknet_image)
    #image = darknet.draw_boxes(detections, image_resized, class_colors)
    image = image_resized
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR), detections
"""
network, class_names, class_colors = darknet.load_network(
                                          './train/yolo-obj-new.cfg',
                                          './train/obj.data',
                                          './train/weights/yolo-obj_last.weights',
                                          1
                                     )
"""
network, class_names, class_colors = darknet.load_network(
                                          './train_back/yolo-obj-new.cfg',
                                          './train_back/obj.data',
                                          './train_back/weights/yolo-obj_last.weights',
                                          1
                                     )

resize_width = darknet.network_width(network)
resize_height = darknet.network_height(network)
center_y_ratio= 218/400
top_camera_f = 600#2.5
board_real_w =  411
conveyor_height = 78
thresh = 0.8
def detect_image_path(image_name = 'data/dog.jpg'):
    
    image, detections = image_detection(
            image_name, network, class_names, class_colors, thresh
            )
    return image, detections

def detect_image(image=cv2.imread('data/dog.jpg')):
    
    darknet_image = darknet.make_image(resize_width, resize_height, 3)

    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image_rgb, (resize_width, resize_height),
                               interpolation=cv2.INTER_LINEAR)

    darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
    detections = darknet.detect_image(network, class_names, darknet_image, thresh=thresh)
    darknet.free_image(darknet_image)
    image = darknet.draw_boxes(detections, image_resized, class_colors)
    image = image_resized
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR),detections
def get_sort_key(l):
  return [l[0],l[2][1],l[2][0]]
def group_by_top_ball(ls,min_x,min_y,ratio,rect_w,rect_h,conveyor_area,center_x,center_y):
  ball_list = []
  x_scale = rect_w/resize_width
  y_scale = rect_h/resize_height
  w1 = rect_w
  d = top_camera_f*(board_real_w/w1)
  real_center_x = (center_x-min_x)*ratio
  real_center_y = (center_y-(min_y+rect_h/2))*ratio
  for l in ls:
    if l[0]=='ball':
      p_x,p_y,w,h = l[2]
      p_x = int(p_x*x_scale)
      p_y = int(p_y*y_scale)
      w = int(w*x_scale)
      h = int(h*y_scale)

      obj_x = int(p_x*ratio)
      obj_y = int((rect_h*center_y_ratio-p_y)*ratio)
      image_obj_x = p_x+min_x
      image_obj_y = p_y+min_y
      R = (w+h)//4
      obj_h = R
      if conveyor_area[0][0]<=image_obj_x<=conveyor_area[0][1] and conveyor_area[1][0]<=image_obj_y<=conveyor_area[1][1]:
        obj_h = conveyor_height + R
        obj_x = int((obj_x-real_center_x)*((d-obj_h)/d)+real_center_x)
        obj_y = int((obj_y-real_center_y)*((d-obj_h)/d)+real_center_y)
        R = int(R*(d-obj_h)/d)
      tmp = [obj_x,obj_y,(image_obj_x,image_obj_y),R,int((w/x_scale+h/y_scale)/2*ratio),obj_h]
      ball_list += [tmp]
  return ball_list
def group_by_side_ball(ls,min_x,min_y,border_y,ratio_x,ratio_y,border_w,roi_height):
  ball_list = []
  ball_side = []
  x_scale = border_w/resize_width
  y_scale = roi_height/resize_height
  for l in ls:
    if l[0]=='ball':
      p_x,p_y,w,h = l[2]
      p_x = int(p_x*x_scale)
      p_y = int(p_y*y_scale)
      w = int(w*x_scale)
      h = int(h*y_scale)
      obj_y = int((border_y-p_y)*ratio_y)
      obj_x = int((border_w-p_x)*ratio_x)
      tmp = [obj_x,obj_y,(p_x+min_x,p_y+min_y),int(w+h)//4]
      ball_side += [[obj_x,obj_y]]
      ball_list += [tmp]
  return ball_list,ball_side
def update_interval(data_x,data_y,target):
  if data_x[0]==-1 or data_x[0]>target[0]-target[2]/2:
    data_x[0] = int(target[0]-target[2]/2)
  if data_x[1]==-1 or data_x[1]<target[0]+target[2]/2:
    data_x[1] = int(target[0]+target[2]/2)
  if data_y[0]==-1 or data_y[0]>target[1]-target[3]/2:
    data_y[0] = int(target[1]-target[3]/2)
  if data_y[1]==-1 or data_y[1]<target[1]+target[3]/2:
    data_y[1] = int(target[1]+target[3]/2)
  return data_x,data_y
def detect_img(img):
  image,detections = detect_image(img)
  cv2.imshow('img',image)
  cv2.waitKey()
def get_top_ball(img,min_x,min_y,ratio,rect_w,rect_h,conveyor_area,center_x,center_y):
  image,detections = detect_image(img)
  balls = group_by_top_ball(sorted(detections,key = get_sort_key),min_x,min_y,ratio,rect_w,rect_h,conveyor_area,center_x,center_y)
  return balls
def get_side_ball(img,min_x,min_y,border_y,ratio_x,ratio_y,border_w,roi_height):
  image,detections = detect_image(img)
  balls,ball_side = group_by_side_ball(sorted(detections,key = get_sort_key),min_x,min_y,border_y,ratio_x,ratio_y,border_w,roi_height)
  return balls,ball_side
if __name__ == "__main__":
    all_pic = glob.glob("./train/images/*.jpg")+glob.glob("./train/images/*.png")
    for pic in all_pic: 
      detect_img(cv2.imread(pic))
