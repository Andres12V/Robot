#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from time import sleep
import cv2 as cv
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import cv2.aruco as aruco
import numpy as np
import sys, time, math
import matplotlib.pyplot as plt

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

class Controller:
    def __init__(self):
        rospy.init_node('control')
        # Camera Image
        self.sub_image = rospy.Subscriber('/camera/image', Image, self.cam_callback)
        self.bridge = CvBridge()
        # Move
        self.sub = rospy.Subscriber('/odom', Odometry, self.control_callback)
        #self.pub = rospy.Publisher('/my_turtlebot_control/cmd_vel', Twist, queue_size=10)
        self.left_wheel_vel = rospy.Publisher('/wheel_left_velocity_controller/command', Float64, queue_size=10)
        self.right_wheel_vel = rospy.Publisher('/wheel_right_velocity_controller/command', Float64, queue_size=10)
        msg1 = Odometry()
        self.l_w_msg = Float64()
        self.r_w_msg = Float64()
        # self.msg = Twist()

        # Infrared sensors
        self.sub_laser1 = rospy.Subscriber('/my_project/sonar1', LaserScan, self.ls1_callback)
        # self.sub_laser2 = rospy.Subscriber('/my_project/sonar2', LaserScan, self.ls2_callback)
        # self.sub_laser3 = rospy.Subscriber('/my_project/sonar3', LaserScan, self.ls3_callback)
        # self.sub_laser4 = rospy.Subscriber('/my_project/sonar4', LaserScan, self.ls4_callback)
        # Position and orientation of the ArUco Marker
        self.pub_aruco_pose = rospy.Publisher('/aruco_pose', Pose, queue_size=10)
        self.aruco_msg = Pose()

        self.rate = rospy.Rate(50)

    def control_callback(self, msg1):
        global Button
        global theta
        global i

        # Get the POSE of the robot
        pos_x = round(msg1.pose.pose.position.x, 2)
        pos_y = round(msg1.pose.pose.position.y, 2)

        rot_q = msg1.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        theta_deg = round(theta*(180/np.pi),2)
        print('Pose: x,y,theta', pos_x, pos_y, theta_deg )

        Flag_r = Flag1

        my_list.append(pos_y)

        cord_x.append(pos_x)
        cord_y.append(pos_y)


        if Button==0:
            # xd = round(self.aruco_msg.position.x, 2)
            #
            # yd = -(round(self.aruco_msg.position.y, 2) - my_list[0])
            xd = 1.0
            yd = 1.0
            # xd = round(tray_x[i], 2)
            # yd = round(tray_y[i], 2)
            print('Desired Pose xd, yd', xd, yd)
        elif Button==1:
            xd = 1.0
            yd = 6.0
            print('Desired Pose xd, yd', xd, yd)
        if i<499:
            i+=1
        else:
            i=499
        K_p = 1  # For ts = 4 sec

        ex = xd-pos_x
        ey = yd-pos_y
        theta_d = np.arctan2(ey, ex)
        e_heading = theta_d - theta

        # Control signals:
        Vel = ( 0.5*np.sqrt( (ex*ex) + (ey*ey) ) )*Flag_r
        Omega = ( 10*np.arctan2( np.sin(e_heading), np.cos(e_heading) ) )*Flag_r
        print(Omega)

        self.l_w_msg = (1/(2*R))*(2*Vel-Omega*L)
        self.r_w_msg = (1/(2*R))*(2*Vel+Omega*L)

        e_t=np.abs(ex)+np.abs(ey)
        if Button==0:
            if e_t==0.0:
                Button = input('Enter 1: ')
                my_list.clear()
        elif Button==1:
            if e_t==0.0:
                Button = 0
                aruco_num = input('Enter ArUco Marker id: ')

        self.left_wheel_vel.publish(self.l_w_msg)
        self.right_wheel_vel.publish(self.r_w_msg)
        self.rate.sleep()

    def cam_callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # ret, frame=cap.read()
            # gray=cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            # cv.imshow('Image', frame)
            # aruco_image = frame

            aruco_image = cv_image
            id_to_find = aruco_num
            marker_size=15 #-[cm]
            #  Get the camera calibration path
            calib_path='/home/andresvergara/turtlebot_ws/src/control/src/Calibpath/'
            camera_matrix= np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
            camera_distortion= np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

            #  180 deg rotation matrix aroud the x axis
            R_flip= np.zeros((3,3), dtype=np.float32)
            R_flip[0,0]=1.0
            R_flip[1,1]=-1.0
            R_flip[2,2]=-1.0

            # Define the aruco dictionary
            aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
            parameters= aruco.DetectorParameters_create()
            font= cv.FONT_HERSHEY_PLAIN
            #Convert in gray scale
            gray=cv.cvtColor(aruco_image, cv.COLOR_BGR2GRAY)

            # FInd all the aruco markers in the image
            corners, ids, rejected=aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
            if ids is not None and ids[0] == id_to_find:
                aruco_image = aruco.drawDetectedMarkers(aruco_image, corners)
                ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
                try:
                    rvec, tvec=ret[0][0,0,:], ret[1][0,0,:]
                    aruco.drawAxis(aruco_image, camera_matrix, camera_distortion, rvec, tvec, 10)
                    str_position= 'Marker Postion x=%4.0f  y=%4.0f  z=%4.0f'%(tvec[0], tvec[1], tvec[2])
                    cv.putText(aruco_image, str_position, (0,290), font, 1, (0, 255, 255), 1, cv.LINE_AA)
                    #-- Obtain the rotation matrix tag->camera
                    R_ct    = np.matrix(cv.Rodrigues(rvec)[0])
                    R_tc    = R_ct.T
                    #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
                    roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)
                    #-- Print the marker's attitude respect to camera frame
                    str_attitude = "Marker Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                        math.degrees(yaw_marker))
                    cv.putText(aruco_image, str_attitude, (0, 315), font, 1, (0, 255, 255), 1, cv.LINE_AA)

                    #-- Now get Position and attitude f the camera respect to the marker
                    pos_camera = -R_tc*np.matrix(tvec).T

                    str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
                    cv.putText(aruco_image, str_position, (0, 340), font, 1, (255, 255, 0), 1, cv.LINE_AA)

                    #-- Get the attitude of the camera respect to the frame
                    roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
                    str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                        math.degrees(yaw_camera))
                    cv.putText(aruco_image, str_attitude, (0, 365), font, 1, (255, 255, 0), 1, cv.LINE_AA)
                    cv.putText(aruco_image, str(aruco_num) , (0, 380), font, 1, (0, 0, 255), 1, cv.LINE_AA)
                except:
                    print('No ArUco Detected')
            # Display the frame
            cv.imshow('frame', aruco_image)
            #cv.imwrite('/home/andresvergara/images_aruco/pics/img15.jpg', cv_image)
            try:
                pos_ar_x = 0.01*tvec[0]
                #print('Aruco_x',pos_ar_x)
                pos_ar_y = 0.01*tvec[2]
                #print('Aruco_y',pos_ar_y)
                self.aruco_msg.position.x = pos_ar_x
                self.aruco_msg.position.y = pos_ar_y
                self.aruco_msg.position.z = 0.5
                self.pub_aruco_pose.publish(self.aruco_msg)
            except:
                pass

        except CvBridgeError as e:
            print(e)
        k = cv.waitKey(1)
        if k == ord('r'):
            self.msg.linear.x = 0
            self.msg.linear.y = 0
            self.msg.angular.z = 0
            rospy.signal_shutdown('shutdown')
            cv.destroyAllWindows()
        #self.pub_image_test.publish(aruco_image2)

        self.rate.sleep()

    def ls1_callback(self, msg_ls1):
        global Flag1
        dist1 = round(msg_ls1.ranges[359], 2)
        phi1 = 0+theta
        if dist1<0.3:
            # Control signals:
            Vel = ( 0.05*np.sqrt( (ex*ex) + (ey*ey) ) )
            Omega= ( 100*-np.arctan2( np.sin(e_heading), np.cos(e_heading) ) )

            self.l_w_msg = (1/(2*R))*(2*Vel-Omega*L)
            self.r_w_msg = (1/(2*R))*(2*Vel+Omega*L)
            Flag1 = 0
        else:
            Flag1 = 1
        self.left_wheel_vel.publish(self.l_w_msg)
        self.right_wheel_vel.publish(self.r_w_msg)



if __name__ == '__main__':
    aruco_num = input('Enter ArUco Marker id: ')
    Flag1 = 0
    Flag2 = 0
    Flag3 = 0
    Flag4 = 0
    Button = 0
    i=0
    R = 0.065
    L = 0.3

    my_list = list()
    cord_x=list()
    cord_y=list()
    tray_x=np.linspace(1.0, 1.0, num=500)
    tray_y=np.linspace(6.0, 0.0, num=500)

    # cap=cv.VideoCapture(0)
    # cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
    # cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

    kt = Controller()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
            plt.plot(1,6, 'x')
            plt.plot(tray_x[499],tray_y[499], 'D')
            plt.plot(cord_x,cord_y, 'g')
            plt.plot(tray_x,tray_y, 'b')
            plt.show()

    except rospy.ROSInterruptException as e:
        print(e)

