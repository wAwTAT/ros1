#!/usr/bin/env python3 
# coding=utf-8
#1.编译器声明和2.编码格式声明
#1:为了防止用户没有将python安装在默认的/usr/bin目录，系统会先从env(系统环境变量)里查找python的安装路径，再调用对应路径下的解析器完成操作，也可以指定python3
#2:Python.X 源码文件默认使用utf-8编码，可以正常解析中文，一般而言，都会声明为utf-8编码

import rospy #引用ROS的Python接口功能包
import cv2, cv_bridge #引用opencv功能包。cv_bridge是ROS图像消息和OpenCV图像之间转换的功能包
import numpy as np #引用数组功能包
from sensor_msgs.msg import Image #引用ROS内的图片消息格式

#定义视觉跟踪类
class Visual:
    def __init__(self):#类初始化
                #从参数服务器获取相关参数，这些参数在launch文件中定义
                self.vertAngle=rospy.get_param('~visual_angle/vertical') #定义摄像头垂直可视角度大小
                self.horizontalAngle=rospy.get_param('~visual_angle/horizontal') #定义摄像头水平可视角度大小
                self.bridge = cv_bridge.CvBridge() #OpenCV与ROS的消息转换类
                self.im_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback) #初始化订阅者,rospy.Publisher()功能是创建订阅者类并输出
                
    #视觉跟踪实现函数
    def callback(self, image_data):
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
                 cv2.drawContours(frame, contours_sorted_red, -1, (0,0,255), 3)           #画出第一个轮廓，因为已排序，所以即为最大的轮廓
                 (x_r, y_r),radius_r = cv2.minEnclosingCircle(contours_sorted_red[0]) #对最大轮廓求最小外接圆，并返回中心坐标(浮点型)，半径(浮点型)
                 x_float=x_r #保留浮点型，用于后面计算角度
                #  y_float=y #保留浮点型，用于后面计算角度
                 x=int(x_r) #转为整型才能用于图像处理
                 y=int(y_r) #转为整型才能用于图像处理
                 radius=int(radius_r) #转为整型才能用于图像处理
                 cv2.circle(frame, (x, y), radius, (0, 0, 255), 3) #画出最小外接圆


                #  #根据区域中心点坐标与摄像头可视角度计算目标与摄像头中心的偏角
                 angleX_r=self.horizontalAngle*(0.5-x_float/frame_width)
                #  angleY=self.vertAngle*(0.5-y_float/frame_height)

                #  angleY_b=self.vertAngle*(0.5-y_float_b/frame_height)

                #  #打印偏角、距离信息
                 rospy.loginfo("RED_angleX:"+str(angleX_r))
                 #rospy.loginfo("angleY:"+str(angleY))
                
                # #没有找到轮廓，速度控制命令置零
                except IndexError:
                 rospy.logwarn("red not found")


                #蓝色
                contours_blue, hierarchy_blue = cv2.findContours(frame_threshold_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                try:
                 contours_sorted_blue=sorted(contours_blue, key=cv2.contourArea, reverse=True)
                 cv2.drawContours(frame, contours_sorted_blue, -1, (255,0,0), 3)
                 (x_b, y_b),radius_b = cv2.minEnclosingCircle(contours_sorted_blue[0])
                 x_float_b=x_b #保留浮点型，用于后面计算角度
                 x_b=int(x_b) #转为整型才能用于图像处理
                 y_b=int(y_b) #转为整型才能用于图像处理
                 radius_b=int(radius_b) #转为整型才能用于图像处理
                 cv2.circle(frame, (x_b, y_b), radius_b, (255, 0, 0), 3) #画出最小外接圆


                #  #根据区域中心点坐标与摄像头可视角度计算目标与摄像头中心的偏角
                 angleX_b=self.horizontalAngle*(0.5-x_float_b/frame_width)
                 angleX_b=self.horizontalAngle*(0.5-x_float_b/frame_width)

                #  #打印偏角、距离信息
                 rospy.loginfo("BLUE_angleX:"+str(angleX_b))
                
                # #没有找到轮廓，速度控制命令置零
                except IndexError:
                 rospy.logwarn("blue not found")
                cv2.imshow("frame_red", frame) #显示识别效果
                cv2.waitKey(1)
           
if __name__ == '__main__': #这段判断的作用是，如果本py文件是直接运行的则判断通过执行if内的内容，如果是import到其他的py文件中被调用(模块重用)则判断不通过
  rospy.init_node("opencv") #创建节点
  rospy.loginfo("OpenCV VisualFollow node started") #打印ROS消息说明节点已开始运行
  Visual_follower=Visual() 
  rospy.spin() 

