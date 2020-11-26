#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    target_zy = cv2.imread(os.path.dirname(sys.argv[0])+'/targettmp_zy.png', 1)
    target_zx = cv2.imread(os.path.dirname(sys.argv[0])+'/targettmp_zx.png', 1)
    wall_zy = cv2.imread(os.path.dirname(sys.argv[0])+'/walltmp_zy.png', 1)
    wall_zx = cv2.imread(os.path.dirname(sys.argv[0])+'/walltmp_zx.png', 1)
    self.targetzytemp = cv2.inRange(target_zy, (200, 200, 200), (255, 255, 255))
    self.targetzxtemp = cv2.inRange(target_zx, (200, 200, 200), (255, 255, 255))
    self.wallzytemp = cv2.inRange(wall_zy, (200, 200, 200), (255, 255, 255))
    self.wallzxtemp = cv2.inRange(wall_zx, (200, 200, 200), (255, 255, 255))
    self.targetzycont, _ = cv2.findContours(self.targetzytemp,2,1)
    self.targetzxcont, _ = cv2.findContours(self.targetzxtemp,2,1)
    self.wallzycont, _ = cv2.findContours(self.wallzytemp,2,1)
    self.wallzxcont, _ = cv2.findContours(self.wallzxtemp,2,1)
    self.scale_zx = 0
    self.scale_zy = 0
    self.scale_xy = 0.038461538461538464
    self.scale_z = 0.04269918660532219
    self.last_blue = np.array([0,0,2.5])
    


    # self.targetChamfer = cv2.distanceTransform(cv2.bitwise_not(self.targettemp),cv2.DIST_L2,0)
    # self.wallzyChamfer = cv2.distanceTransform(cv2.bitwise_not(self.wallzytemp),cv2.DIST_L2,0)
    # self.wallzxChamfer = cv2.distanceTransform(cv2.bitwise_not(self.wallzxtemp),cv2.DIST_L2,0)

    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    self.image_sub1 = rospy.Subscriber("/camera2/robot/image_raw",Image, self.callback2)
    self.image_sub1 = rospy.Subscriber("/camera2/robot/image_raw",Image, self.callback2)

    
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    self.robot_endeffec_pub = rospy.Publisher("/robot/end-effector", Float64MultiArray, queue_size=10)
    self.robot_joints_pub = rospy.Publisher("/robot/joint_states", Float64MultiArray, queue_size=10)
    self.target_joints_pub = rospy.Publisher("/target/joint_states", Float64MultiArray, queue_size=10)
    self.start_time = rospy.get_time()
    self.time_previous_step = rospy.get_time()
    self.error = np.array([0,0,0])
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    self.target_xs = []
    self.target_ys = []
    self.target_zs = []
    self.times = []
    self.elapsed_time = 0

  # Blob Detection From the Labs
  def detect_red_in_image(self,image):
      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(getattr(self, image), (0, 0, 100), (30, 30, 255))
      
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      # mask = cv2.dilate(mask, kernel, iterations=3)
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
      # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      if (M['m00']==0):
        if (image == 'image_zx'):
          M = self.lastRMomzx
        else:
          M = self.lastRMomzy
      if (image == 'image_zx'):
          self.lastRMomzx = M
      else:
        self.lastRMomzy = M

      # Calculate pixel coordinates for the centre of the blob
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      # temp = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
      # temp[cy, cx] = [0,0,255]
      # cv2.imshow('r '+image, temp)
      return np.array([cx, cy])

  # Detecting the centre of the green circle
  def detect_green_in_image(self,image):
      mask = cv2.inRange(getattr(self, image), (0, 100, 0), (80, 255, 80))
      kernel = np.ones((5, 5), np.uint8)
      # mask = cv2.dilate(mask, kernel, iterations=3)
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
      # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
      M = cv2.moments(mask)
      if (M['m00']==0):
        if (image == 'image_zx'):
          M = self.lastGMomzx
        else:
          M = self.lastGMomzy
      if (image == 'image_zx'):
          self.lastGMomzx = M
      else:
        self.lastGMomzy = M
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      # temp = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
      # temp[cy, cx] = [0,0,255]
      # cv2.imshow('g '+image, temp)
      return np.array([cx, cy])

  # Detecting the centre of the blue circle
  def detect_blue_in_image(self,image):
      mask = cv2.inRange(getattr(self, image), (100, 0, 0), (255, 80, 80))
      kernel = np.ones((5, 5), np.uint8)
      # mask = cv2.dilate(mask, kernel, iterations=3)
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
      # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
      M = cv2.moments(mask)
      if (M['m00']==0):
        if (image == 'image_zx'):
          M = self.lastBMomzx
        else:
          M = self.lastBMomzy
      if (image == 'image_zx'):
          self.lastBMomzx = M
      else:
        self.lastBMomzy = M
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      # temp = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
      # temp[cy, cx] = [0,0,255]
      # cv2.imshow('b '+image, temp)
      return np.array([cx, cy])

  # Detecting the centre of the yellow circle
  def detect_yellow_in_image(self,image):
      mask = cv2.inRange(getattr(self, image), (0, 100, 100), (10, 255, 255)) - cv2.inRange(getattr(self, image), (0, 90, 200), (20, 110, 210))
      kernel = np.ones((5, 5), np.uint8)
      # mask = cv2.dilate(mask, kernel, iterations=3)
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
      M = cv2.moments(mask)
      if (M['m00']==0):
        if (image == 'image_zx'):
          M = self.lastYMomzx
        else:
          M = self.lastYMomzy
      if (image == 'image_zx'):
          self.lastYMomzx = M
      else:
        self.lastYMomzy = M
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      # temp = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
      # temp[cy, cx] = [0,0,255]
      # cv2.imshow('y '+image, temp)
      return np.array([cx, cy])

  # Detecting the centre of the yellow circle
  def detect_target_in_image(self,image):
      yellow = cv2.inRange(getattr(self, image), (0, 100, 100), (10, 255, 255)) - cv2.inRange(getattr(self, image), (0, 90, 200), (20, 110, 210))
      kernel = np.ones((5, 5), np.uint8)
      yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernel)
      mask = cv2.inRange(getattr(self, image), (0,52,100),(40,180,255))-yellow
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

      # temp = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
      # mask = cv2.distanceTransform(cv2.bitwise_not(mask),cv2.DIST_L2,0)
      contours, hierarchy = cv2.findContours(mask,2,1)
      print(len(contours))
      scores = []
      centers = []
      for c in range(0,len(contours)):
        
        if (image == "iamge_zy"):
          targetret = cv2.matchShapes(contours[c],self.targetzycont[0],1,0.0)
          wallret = cv2.matchShapes(contours[c],self.wallzycont[0],1,0.0)
        else:
          targetret = cv2.matchShapes(contours[c],self.targetzxcont[0],1,0.0)
          wallret = cv2.matchShapes(contours[c],self.wallzxcont[0],1,0.0)
        m = cv2.moments(contours[c])
        x = int(m['m10']/m['m00'])
        y = int(m['m01']/m['m00'])
        if (targetret < wallret):
            # print(image, targetret)
            # print(m['m00'])
            scores.append(targetret)
            centers.append(np.array([x,y]))
      if (len(scores)>=1):
        scind = np.argmin(scores)
        if (scores[scind]< 0.1):
          cent = centers[scind]
          # temp[cent[1]-3:cent[1]+3,cent[0]-3:cent[0]+3] = [0,0,255]
          # cv2.imshow('o '+image, temp)
          return cent
      
      if (image == 'image_zx'):
          return self.target_zx
      else:
          return self.target_zy

  def detect_wall_in_image(self,image):
      yellow = cv2.inRange(getattr(self, image), (0, 100, 100), (10, 255, 255)) - cv2.inRange(getattr(self, image), (0, 90, 200), (20, 110, 210))
      kernel = np.ones((5, 5), np.uint8)
      yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernel)
      mask = cv2.inRange(getattr(self, image), (0,52,100),(40,180,255))-yellow
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

      # temp = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
      # mask = cv2.distanceTransform(cv2.bitwise_not(mask),cv2.DIST_L2,0)
      contours, hierarchy = cv2.findContours(mask,2,1)
      print(len(contours))
      scores = []
      centers = []
      areas = []
      for c in range(0,len(contours)):
        
        if (image == "iamge_zy"):
          targetret = cv2.matchShapes(contours[c],self.targetzycont[0],1,0.0)
          wallret = cv2.matchShapes(contours[c],self.wallzycont[0],1,0.0)
        else:
          targetret = cv2.matchShapes(contours[c],self.targetzxcont[0],1,0.0)
          wallret = cv2.matchShapes(contours[c],self.wallzxcont[0],1,0.0)
        m = cv2.moments(contours[c])
        x = int(m['m10']/m['m00'])
        y = int(m['m01']/m['m00'])
        if (targetret > wallret):
            print(image, wallret)
            print(m['m00'])
            areas.append(m['m00'])
            scores.append(wallret)
            centers.append(np.array([x,y]))
      if (len(scores)>=1):
        scind = np.argmin(scores)
        print('area:', areas[scind])
        if (scores[scind]< 0.1 and (areas[scind] < 200 or image == 'image_zy')):
          cent = centers[scind]
          print('chosen area:', areas[scind])
          # temp[cent[1]-3:cent[1]+3,cent[0]-3:cent[0]+3] = [0,0,255]
          # cv2.imshow('o '+image, temp)
          return cent
      
      if (image == 'image_zx'):
          return self.wall_zx
      else:
          return self.wall_zy

  def invkin(self,t1, t2, t3, t4):
    st1 = np.sin(t1)
    ct1 = np.cos(t1)
    st2 = np.sin(t2)
    ct2 = np.cos(t2)
    st3 = np.sin(t3)
    ct3 = np.cos(t3)
    st4 = np.sin(t4)
    ct4 = np.cos(t4)
    x = 3*(st1*st2*ct3 + st3*ct1)*ct4 + 3.5*st1*st2*ct3 + 3*st1*st4*ct2 + 3.5*st3*ct1
    y = 3*(st1*st3 - st2*ct1*ct3)*ct4 + 3.5*st1*st3 - 3.5*st2*ct1*ct3 - 3*st4*ct1*ct2
    z = -3*st2*st4 + 3*ct2*ct3*ct4 + 3.5*ct2*ct3 + 2.5
    return np.array([x,y,z])

  # Calculate the conversion from pixel to meter
  def pixel2meter(self,plane):
      # Obtain the centre of each coloured blob
      if (plane=='zx'):
        b = self.blue_zx
        y = self.yellow_zx
      else:
        b = self.blue_zy
        y = self.yellow_zy
      # find the distance between two circles
      dist = self.getLength(y-b)
      return 2.5 / dist

  def get_x(self, Joint):
    jointpos = None
    if (Joint == 1):
      jointpos = self.yellow_zx
    elif (Joint == 2):
      jointpos = self.blue_zx
    elif (Joint == 3):
      jointpos = self.blue_zx
    elif (Joint == 4):
      jointpos = self.green_zx
    elif (Joint == 5):
      jointpos = self.red_zx
    elif (Joint == 'target'):
      jointpos = self.target_zx
    elif (Joint == 'wall'):
      jointpos = self.wall_zx
    vec = jointpos - self.yellow_zx
    return self.scale_zx * vec[0]

  def get_y(self, Joint):
    jointpos = None
    if (Joint == 1):
      jointpos = self.yellow_zy
    elif (Joint == 2):
      jointpos = self.blue_zy
    elif (Joint == 3):
      jointpos = self.blue_zy
    elif (Joint == 4):
      jointpos = self.green_zy
    elif (Joint == 5):
      jointpos = self.red_zy
    elif (Joint == 'target'):
      jointpos = self.target_zy
    elif (Joint == 'wall'):
      jointpos = self.wall_zy
    vec = jointpos - self.yellow_zy
    return self.scale_zy * vec[0]

  def get_z(self, Joint):
    jointpos = None
    if (Joint == 1):
      jointposzy = self.yellow_zy
      jointposzx = self.yellow_zx
    elif (Joint == 2):
      jointposzy = self.blue_zy
      jointposzx = self.blue_zx
    elif (Joint == 3):
      jointposzy = self.blue_zy
      jointposzx = self.blue_zx
    elif (Joint == 4):
      jointposzy = self.green_zy
      jointposzx = self.green_zx
    elif (Joint == 5):
      jointposzy = self.red_zy
      jointposzx = self.red_zx
    elif (Joint == 'target'):
      jointposzy = self.target_zy
      jointposzx = self.target_zx
    elif (Joint == 'wall'):
      jointposzy = self.wall_zy
      jointposzx = self.wall_zx
    z_zy = (self.yellow_zy - jointposzy)[1] * self.scale_z
    z_zx = (self.yellow_zx - jointposzx)[1] * self.scale_z
    return (z_zy + z_zx)/2 #min(z_zy, z_zx)

  def detect_position_in_world(self, Joint):
    x = self.get_x(Joint)
    y = self.get_y(Joint)
    z = self.get_z(Joint)
    return np.array([x,y,z])

  def getLength(self, vec):
    return np.linalg.norm(vec)

  def angleBetweenVecs(self, a, b):
    return np.arccos(np.dot(a,b)/(self.getLength(a)*self.getLength(b)))

  def projToPlane(self, vec, norm):
    return vec - (np.dot(vec, norm)*norm)

  def projToVec(self, vec1, vec2):
    return (np.dot(vec1,vec2)/self.getLength(vec2)**2 )*vec2

  def angleToPlane(self, vec, norm):
    return (np.pi/2)-self.angleBetweenVecs(vec, norm)

  def trajectory(self):
    # get current time
    cur_time = rospy.get_time()-self.start_time
    Joint2 = float((np.pi/2)*np.sin((np.pi/15)*cur_time))
    Joint3 = float((np.pi/2)*np.sin((np.pi/18)*cur_time))
    Joint4 = float((np.pi/2)*np.sin((np.pi/20)*cur_time))
    return Joint2, Joint3, Joint4

  def get_data_from_images(self):
    self.red_zx = self.detect_red_in_image("image_zx")
    self.red_zy = self.detect_red_in_image("image_zy")
    self.blue_zx = self.detect_blue_in_image("image_zx")
    self.blue_zy = self.detect_blue_in_image("image_zy")
    self.green_zx = self.detect_green_in_image("image_zx")
    self.green_zy = self.detect_green_in_image("image_zy")
    self.yellow_zx = self.detect_yellow_in_image("image_zx")
    self.yellow_zy = self.detect_yellow_in_image("image_zy")
    self.target_zx = self.detect_target_in_image("image_zx")
    self.target_zy = self.detect_target_in_image("image_zy")
    self.wall_zx = self.detect_wall_in_image("image_zx")
    self.wall_zy = self.detect_wall_in_image("image_zy")

    if (self.scale_zx == 0):
      self.scale_zy = self.pixel2meter('zy')
      self.scale_zx = self.pixel2meter('zx')

  def get_Joint_world_positions(self):
    self.yellow_pos = self.detect_position_in_world(1)
    self.blue_pos = self.detect_position_in_world(2)
    self.green_pos = self.detect_position_in_world(4)
    self.red_pos = self.detect_position_in_world(5)
    print('yellow',self.yellow_pos)
    print('blue',self.blue_pos)
    print('green',self.green_pos)
    print('red',self.red_pos)
    self.target_pos = self.detect_position_in_world('target')
    self.wall_pos = self.detect_position_in_world('wall')
    self.link3 = self.green_pos-np.array([0,0,2.5])
    self.link4 = self.red_pos-self.green_pos

  def predict_Joint_angles(self):
    m3 = self.angleToPlane(self.link3,np.array([1, 0, 0]))
    self.link3[2] = max(self.link3[2], 0)
    m2 =(-np.arctan((self.link3[1])/(self.link3[2])))
    rot = np.array([[np.cos(m3), 0,np.sin(m3)],[0,1,0],[-np.sin(m3),0,np.cos(m3)]]) 
    m2 = self.angleBetweenVecs(self.blue_pos - np.array([0,0,2.5]), np.array([0,0,1]))

    if (self.green_pos[1] > 0):
      m2 *= -1

    link3proj = self.projToPlane(self.link3,np.array([0,1,0]))
    print(link3proj)

    m4 = self.angleBetweenVecs(self.link3, self.link4)
    if (self.green_pos[1] > 0):
      if (self.red_pos[2]< self.green_pos[2]):
        m4 *= -1
    else:
      if (self.red_pos[2]>self.green_pos[2]):
        m4 *= -1
    return np.array([m2,m3,m4])

  def draw_centers_on_images(self):
    self.image_zx[self.red_zx[1], self.red_zx[0]] = [255,255,255]
    self.image_zx[self.blue_zx[1], self.blue_zx[0]] = [255,255,255]
    self.image_zx[self.yellow_zx[1], self.yellow_zx[0]] = [255,255,255]
    self.image_zx[self.green_zx[1], self.green_zx[0]] = [255,255,255]
    self.image_zx[self.target_zx[1], self.target_zx[0]] = [255,255,255]
    self.image_zx[self.wall_zx[1], self.wall_zx[0]] = [255,255,255]

    self.image_zy[self.wall_zy[1], self.wall_zy[0]] = [255,255,255]
    self.image_zy[self.target_zy[1], self.target_zy[0]] = [255,255,255]
    self.image_zy[self.red_zy[1], self.red_zy[0]] = [255,255,255]
    self.image_zy[self.blue_zy[1], self.blue_zy[0]] = [255,255,255]
    self.image_zy[self.yellow_zy[1], self.yellow_zy[0]] = [255,255,255]
    self.image_zy[self.green_zy[1], self.green_zy[0]] = [255,255,255]
  
  def calculate_jacobian(self, angles):
    st1 = np.sin(0)
    ct1 = np.cos(0)
    st2 = np.sin(angles[0])
    ct2 = np.cos(angles[0])
    st3 = np.sin(angles[1])
    ct3 = np.cos(angles[1])
    st4 = np.sin(angles[2])
    ct4 = np.cos(angles[2])
    row1 = [0, 3*ct3*ct4+3.5*ct3, -3*st3*st4]
    row2 = [3*st2*st4-3*ct2*ct3*ct4-3.5*ct2*ct3, 3*st2*st3*ct4+3.5*st2*st3, 3*st2*st4*ct3-3*ct2*ct4]
    row3 = [-3*st2*ct3*ct4-3.5*st2*ct3-3*st4*ct2, -3*st3*ct2*ct4-3.5*st3*ct2, -3*st2*ct4-3*st4*ct2*ct3]
    return np.array([row1, row2, row3])

  def control_closed(self):
    # P gain
    K_p = np.array([[20 ,0,0],[0,20,0],[0,0,20]])
    # D gain
    K_d = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.1]])
    # estimate time step
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    q = self.predict_Joint_angles() # estimate initial value of joints'
    # robot end-effector position
    pos = self.red_pos
    # desired trajectory
    pos_d = self.target_pos
    # estimate derivative of error
    self.error_d = ((pos_d - pos) - self.error)/dt
    # estimate error
    self.error = pos_d-pos
    
    J_inv = np.linalg.pinv(self.calculate_jacobian(q))  # calculating the psudeo inverse of Jacobian
    t1 = np.dot(K_d,self.error_d.transpose())
    t2 = np.dot(K_p,self.error.transpose())
    t = ( t1 + t2 )
    dq_d = np.dot(J_inv, t)  # control input (angular velocity of joints)
    q_d = q + (dt * dq_d)  # control input (angular position of joints)
    return q_d

  def control_null(self):
    # P gain
    K_p = np.array([[10,0,0],[0,10,0],[0,0,10]])
    # D gain
    K_d = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.1]])
    # estimate time step
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    # robot end-effector position
    pos = self.red_pos 
    # desired trajectory
    pos_d = self.target_pos
    # estimate derivative of error
    self.error_d = ((pos_d - pos) - self.error)/dt
    # estimate error
    self.error = pos_d-pos
    q = self.predict_Joint_angles() # estimate initial value of joints'
    J = self.calculate_jacobian(q)
    J_inv = np.linalg.pinv(J)  # calculating the psudeo inverse of Jacobian
    t1 = np.dot(K_d,self.error_d.transpose())
    t2 = np.dot(K_p,self.error.transpose())
    t = ( t1 + t2 )
    dq_d = np.dot(J_inv, t)  # control input (angular velocity of joints)

    wall_d = self.wall_pos
    self.error_w_d = ((wall_d - pos) - self.error)/dt
    self.error_w = wall_d-pos
    null = np.eye(3) - np.dot(J_inv, J)
    second_task = np.dot(null, self.error_w_d.transpose())

    q_d = q + (dt * (dq_d - second_task))  # control input (angular position of joints)
    return q_d

  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.image_zy = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # self.image_zy = cv2.GaussianBlur(self.image_zy, (5,5),0)

    self.elapsed_time = rospy.get_time() - self.start_time
    
    self.get_data_from_images()
    self.get_Joint_world_positions()

    self.targetposs = Float64MultiArray()
    self.targetposs.data = self.target_pos
    self.target_joints_pub.publish(self.targetposs)

    self.endeffect = Float64MultiArray()
    self.endeffect.data = self.red_pos
    self.robot_endeffec_pub.publish(self.endeffect)

    print("end effector:")
    print(self.endeffect.data)


    # print('measured', self.red_pos)
    # print('predicted', self.invkin(0,0.1,0.1,0))
    j2, j3, j4 = self.trajectory()
    # joints = self.control_closed()
    # j2 = joints[0]
    # j3 = joints[1]
    # j4 = joints[2]
    print("expected")
    print(j2, j3, j4)

    self.joints = Float64MultiArray()
    self.joints.data = self.predict_Joint_angles()
    self.robot_joints_pub.publish(self.joints)
    print("measured")
    print(self.joints.data)

    self.Joint2 = Float64()
    self.Joint2.data = j2 
    # self.Joint2.data = 0.0
    self.robot_joint2_pub.publish(self.Joint2)
    self.Joint3 = Float64()
    self.Joint3.data = j3
    # self.Joint3.data = 1.5
    self.robot_joint3_pub.publish(self.Joint3)
    self.Joint4 = Float64()
    self.Joint4.data = j4
    # self.Joint4.data = 0
    self.robot_joint4_pub.publish(self.Joint4)
    

    self.draw_centers_on_images()

    im1 = cv2.imshow('window1_zy', self.image_zy)
    im2 = cv2.imshow('window2_zx', self.image_zx)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.image_zy, "bgr8"))
    except CvBridgeError as e:
      print(e)

  # Recieve and store second image
  def callback2(self,data):
    # Recieve the image
    try:
      self.image_zx = self.bridge.imgmsg_to_cv2(data, "bgr8")
      # self.image_zx = cv2.GaussianBlur(self.image_zx, (5,5),0)
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)