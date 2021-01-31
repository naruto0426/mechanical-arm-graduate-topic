import darknet
import cv2
import random
import glob
import numpy as np
def image_detection(image_path, network, class_names, class_colors, thresh):
    # Darknet doesn't accept numpy images.
    # Create one with image we reuse for each detect
    width = darknet.network_width(network)
    height = darknet.network_height(network)
    darknet_image = darknet.make_image(width, height, 3)

    image = cv2.imread(image_path)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image_rgb, (width, height),
                               interpolation=cv2.INTER_LINEAR)

    darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
    detections = darknet.detect_image(network, class_names, darknet_image, thresh=thresh)
    darknet.free_image(darknet_image)
    image = darknet.draw_boxes(detections, image_resized, class_colors)
    image = image_resized
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR), detections
network, class_names, class_colors = darknet.load_network(
                                          './train/yolo-obj-new.cfg',
                                          './train/obj.data',
                                          './train/weights/yolo-obj_last.weights',
                                          1
                                      )
thresh = 0.1
def detect_image_path(image_name = 'data/dog.jpg'):
    
    image, detections = image_detection(
            image_name, network, class_names, class_colors, thresh
            )
    return image, detections

def detect_image(image=cv2.imread('data/dog.jpg')):
    
    width = darknet.network_width(network)
    height = darknet.network_height(network)
    darknet_image = darknet.make_image(width, height, 3)

    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image_rgb, (width, height),
                               interpolation=cv2.INTER_LINEAR)

    darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
    detections = darknet.detect_image(network, class_names, darknet_image, thresh=thresh)
    darknet.free_image(darknet_image)
    image = darknet.draw_boxes(detections, image_resized, class_colors)
    image = image_resized
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR),detections
def get_sort_key(l):
  return [l[0],l[2][1],l[2][0]]
def group_by_top_ball(ls,min_x,min_y,ratio,rect_h):
  ball_list = []
  for l in ls:
    if l[0]=='ball':
      p_x,p_y,w,h = [int(ele) for ele in l[2]]
      print(p_x,p_y,w*ratio,h*ratio)
      obj_x = int(p_x*ratio)
      obj_y = int((rect_h*181/400-p_y)*ratio)
      tmp = [obj_x,obj_y,(p_x+min_x,p_y+min_y),(w+h)//4,int((w+h)//2*ratio)]

      ball_list += [tmp]
  return ball_list
def group_by_side_ball(ls,min_x,min_y,border_y,ratio_x,ratio_y,border_w):
  ball_list = []
  ball_side = []
  for l in ls:
    if l[0]=='ball':
      p_x,p_y,w,h = [int(ele) for ele in l[2]]
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
  print(detections)
  cv2.imshow('111',cv2.resize(image, (500, 500),
                               interpolation=cv2.INTER_LINEAR))
def get_top_ball(img,min_x,min_y,ratio,rect_h):
  image,detections = detect_image(img)
  balls = group_by_top_ball(sorted(detections,key = get_sort_key),min_x,min_y,ratio,rect_h)
  return balls
def get_side_ball(img,min_x,min_y,border_y,ratio_x,ratio_y,border_w):
  image,detections = detect_image(img)
  balls,ball_side = group_by_side_ball(sorted(detections,key = get_sort_key),min_x,min_y,border_y,ratio_x,ratio_y,border_w)
  return balls,ball_side
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    while 1:
      ret, img = cap.read() 
      detect_img(img)
      key = cv2.waitKey(2)
      if key == ord("q"):
        break
      elif key==ord("w"):
        count = len(glob.glob("./train/images/fail_*.jpg"))
        cv2.imwrite(f'train/images/fail_{count}.jpg',img)
