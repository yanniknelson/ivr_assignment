#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    self.image_sub1 = rospy.Subscriber("/camera2/robot/image_raw",Image, self.callback2)
    
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    self.robot_joints_pub = rospy.Publisher("/robot/joint_states", Float64MultiArray, queue_size=10)
    self.start_time = rospy.get_time()
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

  # Blob Detection From the Labs
  def detect_red_in_image(self,image):
      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(image, (0, 0, 100), (30, 30, 255))
      
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      # mask = cv2.dilate(mask, kernel, iterations=3)
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
      # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      if (M['m00']==0):
        M = self.lastRMom
      self.lastRMom = M
      # Calculate pixel coordinates for the centre of the blob
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      # temp = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
      # temp[cy, cx] = [0,0,255]
      # cv2.imshow('r '+image, temp)
      return np.array([cx, cy])

  # Detecting the centre of the green circle
  def detect_green_in_image(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (80, 255, 80))
      kernel = np.ones((5, 5), np.uint8)
      # mask = cv2.dilate(mask, kernel, iterations=3)
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
      # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
      M = cv2.moments(mask)
      if (M['m00']==0):
        M = self.lastGMom
      self.lastGMom = M
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      # temp = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
      # temp[cy, cx] = [0,0,255]
      # cv2.imshow('g '+image, temp)
      return np.array([cx, cy])

  # Detecting the centre of the blue circle
  def detect_blue_in_image(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 80, 80))
      kernel = np.ones((5, 5), np.uint8)
      # mask = cv2.dilate(mask, kernel, iterations=3)
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
      # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
      M = cv2.moments(mask)
      if (M['m00']==0):
        M = self.lastBMom
      self.lastBMom = M
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      # temp = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
      # temp[cy, cx] = [0,0,255]
      # cv2.imshow('b '+image, temp)
      return np.array([cx, cy])

  # Detecting the centre of the yellow circle
  def detect_yellow_in_image(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      # mask = cv2.dilate(mask, kernel, iterations=3)
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
      # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
      M = cv2.moments(mask)
      if (M['m00']==0):
        M = self.lastYMom
      self.lastYMom = M
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      # temp = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
      # temp[cy, cx] = [0,0,255]
      # cv2.imshow('y '+image, temp)
      return np.array([cx, cy])

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
    vec = jointpos - self.yellow_zy
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
    vec = jointpos - self.yellow_zy
    return self.scale_zy * vec[0]

  def get_z(self, Joint):
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
    vec = self.yellow_zy - jointpos
    return self.scale_zy * vec[1]

  def detect_position_in_world(self, Joint):
    x = self.get_x(Joint)
    y = self.get_y(Joint)
    z = self.get_z(Joint)
    return np.array([x,y,z])

  def getLength(self, vec):
    return np.sqrt(np.sum(vec**2))
    
  def angleBetweenVecs(self, a, b):
    return np.arccos(np.dot(a,b)/(self.getLength(a)*self.getLength(b)))

  def trajectory(self):
    # get current time
    cur_time = np.array([rospy.get_time()-self.start_time])
    Joint2 = float((np.pi/2)*np.sin((np.pi/15)*cur_time))
    Joint3 = float((np.pi/2)*np.sin((np.pi/18)*cur_time))
    Joint4 = float((np.pi/2)*np.sin((np.pi/20)*cur_time))
    return Joint2, Joint3, Joint4

  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.image_zy = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    self.image_zy = cv2.GaussianBlur(self.image_zy, (5,5),0)

    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    self.red_zx = self.detect_red_in_image(self.image_zx)
    self.red_zy = self.detect_red_in_image(self.image_zy)
    self.blue_zx = self.detect_blue_in_image(self.image_zx)
    self.blue_zy = self.detect_blue_in_image(self.image_zy)
    self.green_zx = self.detect_green_in_image(self.image_zx)
    self.green_zy = self.detect_green_in_image(self.image_zy)
    self.yellow_zx = self.detect_yellow_in_image(self.image_zx)
    self.yellow_zy = self.detect_yellow_in_image(self.image_zy)

    self.scale_zy = self.pixel2meter('zy')
    self.scale_zx = self.pixel2meter('zx')

    self.yellow_pos = self.detect_position_in_world(1)
    self.blue_pos = self.detect_position_in_world(2)
    self.green_pos = self.detect_position_in_world(4)
    self.red_pos = self.detect_position_in_world(5)
    
    self.link1 = self.blue_pos-self.yellow_pos
    self.link3 = self.green_pos-self.blue_pos
    self.link4 = self.red_pos-self.green_pos

    # print(self.getLength(self.link1), self.getLength(self.link3), self.getLength(self.link4))
    m2 = (np.pi/2)-self.angleBetweenVecs(np.array([0,-1, 0]), self.link3)
    m3 = (np.pi/2)-self.angleBetweenVecs(np.array([-1, 0, 0]), self.link3)
    m4 = self.angleBetweenVecs(self.link3, self.link4)

    

    j2, j3, j4 = self.trajectory()
    print(j2, j3, j4)
    print(m2, m3, m4)

    t = [j2, j3, j4]

    self.MeasuredJoint2 = Float64()
    self.MeasuredJoint2.data = m2
    self.MeasuredJoint3 = Float64()
    self.MeasuredJoint3.data = m3
    self.MeasuredJoint4 = Float64()
    self.MeasuredJoint4.data = m4

    j = [m2,m3,m4]

    # print(np.sum(np.array(t)-np.array(j))**2/3)

    self.joints = Float64MultiArray()
    self.joints.data = j

    self.Joint2 = Float64()
    self.Joint2.data = j2
    self.Joint3 = Float64()
    self.Joint3.data = j3
    self.Joint4 = Float64()
    self.Joint4.data = j4

    # print("angle from xz plane")
    # print((np.pi/2)-self.angleBetweenVecs(np.array([0,-1,0]), self.detect_position_in_world(4)-self.detect_position_in_world(2)))
    # print("angle from xy plane")
    # print((np.pi/2)-self.angleBetweenVecs(np.array([1,0,0]), self.detect_position_in_world(4)-self.detect_position_in_world(2)))

    im1 = cv2.imshow('window1', self.image_zy)
    im2 = cv2.imshow('window2', self.image_zx)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.image_zy, "bgr8"))
      self.robot_joint2_pub.publish(self.Joint2)
      self.robot_joint3_pub.publish(self.Joint3)
      self.robot_joint4_pub.publish(self.Joint4)
      self.robot_joints_pub.publish(self.joints)
    except CvBridgeError as e:
      print(e)

  # Recieve and store second image
  def callback2(self,data):
    # Recieve the image
    try:
      self.image_zx = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.image_zx = cv2.GaussianBlur(self.image_zx, (5,5),0)
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