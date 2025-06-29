#!/usr/bin/env python3 
# coding=utf-8
#1.编译器声明和2.编码格式声明
#1:为了防止用户没有将python安装在默认的/usr/bin目录，系统会先从env(系统环境变量)里查找python的安装路径，再调用对应路径下的解析器完成操作，也可以指定python3
#2:Python.X 源码文件默认使用utf-8编码，可以正常解析中文，一般而言，都会声明为utf-8编码

from logging import Handler
import math
from numpy.core.fromnumeric import size
from yaml import scan
import rospy #引用ROS的Python接口功能包
import cv2, cv_bridge #引用opencv功能包。cv_bridge是ROS图像消息和OpenCV图像之间转换的功能包
import numpy as np
from rospy import rostime #引用数组功能包
from sensor_msgs.msg import Image ,LaserScan#引用ROS内的图片消息格式
import message_filters #https://blog.csdn.net/chishuideyu/article/details/77479758
from geometry_msgs.msg import PoseStamped,TransformStamped#引用ROS内的图片消息格式
import tf
import tf2_py
import tf2_ros
#定义视觉跟踪类
dis_sum = 0   #距离求和
a = 0         #计算合法计数器
angleX_r = 0  #红色锥桶相对于小车的角度
angleX_b = 0  #蓝色锥桶相对于小车的角度
dis_r = 0     #红色锥桶相对于小车的距离
dis_b = 0     #蓝色锥桶相对于小车的距离
#笛卡尔坐标系下锥桶坐标
x_dk_r = 0
y_dk_r = 0
x_dk_b = 0
y_dk_b = 0
PI = 3.1416

class Visual:
    def __init__(self):#类初始化
                #从参数服务器获取相关参数，这些参数在launch文件中定义
                self.vertAngle=rospy.get_param('~visual_angle/vertical') #定义摄像头垂直可视角度大小
                self.horizontalAngle=rospy.get_param('~visual_angle/horizontal') #定义摄像头水平可视角度大小
                self.dis_limit=rospy.get_param('~visual_angle/dis_limit') #超出该距离将视为无效
                self.angle_limit=rospy.get_param('~visual_angle/angle_limit') #定义摄像头水平可视角度大小

                self.bridge = cv_bridge.CvBridge() #OpenCV与ROS的消息转换类

                im_sub = message_filters.Subscriber('/usb_cam/image_raw', Image)
                scan_sub = message_filters.Subscriber('/scan_1', LaserScan)
                self.point_pub = rospy.Publisher("point_mid",PoseStamped,queue_size=10)

                self.point_mid = PoseStamped()
                self.point_mid.header.frame_id = "base_link"
                

                self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, scan_sub],10, 0.5, allow_headerless=True)
                self.timeSynchronizer.registerCallback(self.callback)
    #视觉跟踪实现函数
    def callback(self, image_data, scan_date):
                def git_dis(angle):
                  global dis_sum,a
                  if angle >= self.angle_limit:
                    for i in scan_date.ranges[int(4 * (angle - self.angle_limit)):int(4 * (angle + self.angle_limit))]:
                      if i <= self.dis_limit and i >= 0:
                        dis_sum = (dis_sum + i)
                        a = (a + 1)
                    if a == 0:
                      return 50
                    return (dis_sum / a)
                  
                  elif  angle <= -self.angle_limit:
                    for i in scan_date.ranges[1440 + int(4 * (angle - self.angle_limit)):1440 + int(4 * (angle + self.angle_limit))]:
                      if i <= self.dis_limit and i >= 0:
                        dis_sum = (dis_sum + i)
                        a = (a + 1)
                    if a == 0:
                      return 50
                    return (dis_sum / a)

                  else :
                    for i in scan_date.ranges[0:8]:
                      if i <= self.dis_limit and i >= 0:
                        dis_sum = (dis_sum + i)
                        a = (a + 1)
                    if a == 0:
                      return 50
                    return (dis_sum / a)
                
                global dis_sum,a,angleX_r,angleX_b,dis_r,dis_b,x_dk_r,y_dk_r,x_dk_b,y_dk_b,PI
                #图像转换与预处理
                frame = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')            #ROS图像转OpenCV图像
                frame = cv2.resize(frame, (320,240), interpolation=cv2.INTER_AREA)            #降低图像分辨率，以提高程序运行速度            
                frame_height, frame_width, frame_channels = frame.shape #获得图像尺寸高度、宽度、通道数
                #图像二值化
                # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                red_lower=np.array([0, 0, 146])  #红色
                red_upper=np.array([121, 73, 255]) #红色
                
                blue_lower = np.array([78, 25, 0]) #蓝色
                blue_upper = np.array([162, 79, 40]) #蓝色

                frame_threshold_red = cv2.inRange(frame, red_lower, red_upper)
                frame_threshold_blue = cv2.inRange(frame, blue_lower, blue_upper)
                # #膨胀腐蚀
                kernel_red = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
                frame_threshold_red = cv2.erode(frame_threshold_red, kernel_red)
                frame_threshold_red = cv2.dilate(frame_threshold_red, kernel_red)
                frame_threshold_red = cv2.dilate(frame_threshold_red, kernel_red)
                frame_threshold_red = cv2.erode(frame_threshold_red, kernel_red)

                kernel_blue = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
                frame_threshold_blue = cv2.erode(frame_threshold_blue, kernel_blue)
                frame_threshold_blue = cv2.dilate(frame_threshold_blue, kernel_blue)
                frame_threshold_blue = cv2.dilate(frame_threshold_blue, kernel_blue)
                frame_threshold_blue = cv2.erode(frame_threshold_blue, kernel_blue)

                # #寻找轮廓API，功能：输入图片，返回原图、轮廓对象、轮廓父子结构
                contours_red, hierarchy = cv2.findContours(frame_threshold_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #cv2.RETR_EXTERNAL：只检测外轮廓，cv2.CHAIN_APPROX_SIMPLE)：只保留轮廓的交点
                # #如果找到了轮廓，则保留最大面积轮廓—>规划一个比轮廓形状小的区域—>获得该区域的深度信息(即与目标的距离)—>从深度信息、区域中心坐标判断并给twist赋值以进行前进后退左右转
                try:
                 contours_sorted_red=sorted(contours_red, key=cv2.contourArea, reverse=True) #按照轮廓面积进行排序
                 (x_r, y_r),radius_r = cv2.minEnclosingCircle(contours_sorted_red[0]) #对最大轮廓求最小外接圆，并返回中心坐标(浮点型)，半径(浮点型)
                 x_float=x_r #保留浮点型，用于后面计算角度
                #  #根据区域中心点坐标与摄像头可视角度计算目标与摄像头中心的偏角
                 angleX_r=self.horizontalAngle*(0.5-x_float/frame_width)
                #  #打印偏角、距离信息
                 dis_r = git_dis(angleX_r)
                 a = 0
                 dis_sum = 0
                 x_dk_r = dis_r * math.cos(angleX_r * PI / 180)
                 y_dk_r = dis_r * math.sin(angleX_r * PI / 180)
                 rospy.loginfo("RED_angleX:"+str(angleX_r)+"      RED_dis:"+str(dis_r)+"    x:"+str(x_dk_r)+"    y:"+str(y_dk_r))
                 
                # #没有找到轮廓，速度控制命令置零
                except IndexError:
                 rospy.logwarn("没有发现红色锥桶")
                 angleX_r = 0
                 dis_r = 0

                #蓝色
                contours_blue, hierarchy_blue = cv2.findContours(frame_threshold_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                try:
                 contours_sorted_blue=sorted(contours_blue, key=cv2.contourArea, reverse=True)
                 (x_b, y_b),radius_b = cv2.minEnclosingCircle(contours_sorted_blue[0])
                 x_float_b=x_b #保留浮点型，用于后面计算角度
                #  #根据区域中心点坐标与摄像头可视角度计算目标与摄像头中心的偏角
                 angleX_b=self.horizontalAngle*(0.5-x_float_b/frame_width)
                #  #打印偏角、距离信息
                 dis_b = git_dis(angleX_b)
                 a = 0
                 dis_sum = 0
                 x_dk_b = dis_b * math.cos(angleX_b * PI / 180)
                 y_dk_b = dis_b * math.sin(angleX_b * PI / 180)
                 rospy.loginfo("BLUE_angleX:"+str(angleX_b)+"      BLUE_dis:"+str(dis_b)+"    x:"+str(x_dk_b)+"    y:"+str(y_dk_b))

                # #没有找到轮廓，速度控制命令置零
                except IndexError:
                 rospy.logwarn("没有发现蓝色锥桶")
                 angleX_b = 0
                 dis_b = 0

                if dis_r != 0 and dis_b != 0:
                  self.point_mid.pose.position.x = (x_dk_r + x_dk_b)/2
                  self.point_mid.pose.position.y = (y_dk_r + y_dk_b)/2
                  self.point_mid.pose.position.z = 0
                  # self.point_mid.header.stamp = rostime.Time.now()

                  q = tf.transformations.quaternion_from_euler(0,0,-(x_dk_r - x_dk_b) / (y_dk_r - y_dk_b))
                  if y_dk_r == y_dk_b :
                    return 0
                  else:
                    self.point_mid.pose.orientation.x = q[0]
                    self.point_mid.pose.orientation.y = q[1]
                    self.point_mid.pose.orientation.z = q[2]
                    self.point_mid.pose.orientation.w = q[3]
                    self.point_pub.publish(self.point_mid)
                    rospy.loginfo("中点坐标发送成功   X："+ str(self.point_mid.pose.position.x)+"    Y:"+str(self.point_mid.pose.position.y)+"   theta:"+str(-(x_dk_r - x_dk_b) / (y_dk_r - y_dk_b)))


if __name__ == '__main__': #这段判断的作用是，如果本py文件是直接运行的则判断通过执行if内的内容，如果是import到其他的py文件中被调用(模块重用)则判断不通过
  rospy.init_node("opencv") #创建节点
  rospy.loginfo("opencv节点开启成功") #打印ROS消息说明节点已开始运行
  Visual_follower=Visual() 
  rospy.spin() 

