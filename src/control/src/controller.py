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
        # Camera Image topic
        self.sub_image = rospy.Subscriber('/camera/image', Image, self.cam_callback)
        self.bridge = CvBridge()
        # POSE of the robot
        self.sub = rospy.Subscriber('/odom', Odometry, self.control_callback)
        #self.pub = rospy.Publisher('/my_turtlebot_control/cmd_vel', Twist, queue_size=10)
        # Wheel velocity topics
        self.left_wheel_vel = rospy.Publisher('/wheel_left_velocity_controller/command', Float64, queue_size=10)
        self.right_wheel_vel = rospy.Publisher('/wheel_right_velocity_controller/command', Float64, queue_size=10)
        msg1 = Odometry()
        self.l_w_msg = Float64()
        self.r_w_msg = Float64()
        # self.msg = Twist()

        # Infrared sensors
        # self.sub_laser1 = rospy.Subscriber('/my_project/sonar1', LaserScan, self.ls1_callback)
        # self.sub_laser2 = rospy.Subscriber('/my_project/sonar2', LaserScan, self.ls2_callback)
        # self.sub_laser3 = rospy.Subscriber('/my_project/sonar3', LaserScan, self.ls3_callback)
        # self.sub_laser4 = rospy.Subscriber('/my_project/sonar4', LaserScan, self.ls4_callback)
        # Position and orientation of the ArUco Marker
        self.pub_aruco_pose = rospy.Publisher('/aruco_pose', Pose, queue_size=10)
        self.aruco_msg = Pose()

        # Frecuency of the loop:
        self.rate = rospy.Rate(100)

    def control_callback(self, msg1):
        global Button
        global theta
        global i
        global int_e
        global smallest

        # Get the POSE of the robot:
        pos_x = round(msg1.pose.pose.position.x, 2)
        pos_y = round(msg1.pose.pose.position.y, 2)
        rot_q = msg1.pose.pose.orientation

        # Get the euler angles:
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        theta_deg = round(theta*(180/np.pi),2)
        print('Pose: x,y,theta', pos_x, pos_y, theta_deg)

        # Flags that switch the go-to-goal behavior to obstacle avoidance behavior:
        Flag_r = 1
        my_list.append(pos_y)

        # Append the position of the robot in each iteration for the plot
        cord_x.append(pos_x)
        cord_y.append(pos_y)

        if Button==0:
            try:
                # xd = 2.4
                # yd = 0.0
                # If the marker is detected reach the marker
                if state=='M_detected':
                    if id_to_find ==12:
                        xd = 2.4
                        yd = -1.0
                    if id_to_find ==13:
                        xd = 2.4
                        yd = -4.5
                    if id_to_find ==14:
                        xd = 2.4
                        yd = -8.0
                    if id_to_find ==15:
                        xd = -0.2
                        yd = -1.0
                    if id_to_find ==16:
                        xd = -0.2
                        yd = -4.5
                    if id_to_find ==17:
                        xd = -0.2
                        yd = -8.0
                    if id_to_find ==18:
                        xd = -2.8
                        yd = -1.0
                    if id_to_find ==21:
                        xd = -2.8
                        yd = -4.5
                    if id_to_find ==25:
                        xd = -2.8
                        yd = -8.0
                # If there is no marker detected follow the designed path
                if state=='No_M_detected':
                    xd = round(my_listx[i], 2)
                    yd = round(my_listy[i], 2)
            except:
                pass
            # xd = round(self.aruco_msg.position.x, 2)
            # yd = -(round(self.aruco_msg.position.y, 2) - my_list[0])
            # xd = 2.0
            # yd = 2.0
            # xd = round(my_listx[i], 2)
            # yd = round(my_listy[i], 2)
            #print('Desired Pose xd, yd', xd, yd)
        elif Button==1:
            xd = 1.0
            yd = 6.0
            print('Desired Pose xd, yd', xd, yd)
        print('--------',state)
        ex = xd-pos_x
        ey = yd-pos_y
        for it_x in my_listx:
            for it_y in my_listy:
                e_x = it_x-pos_x
                e_y = it_y-pos_y
                e_d = np.sqrt( (e_x*e_x) + (e_y*e_y) )
                if smallest is None:
                    smallest=e_d
                elif e_x<smallest:
                    smallest=e_d 


        # Calculation of the total error in X and Y positions
        e_t=np.abs(ex)+np.abs(ey)

        if state=='M_detected':
            if e_t==0:
                theta_d=theta
            else:
                theta_d = np.arctan2(ey, ex)
        if state=='No_M_detected':
            if e_t==0:
                theta_d=theta
            else:
                theta_d = np.arctan2(ey, ex)

        e_heading = (theta_d - theta)
        print('Desired Posew xd, yd', xd, yd, theta_d)

        # Control signals:
        e_diag = np.sqrt( (ex*ex) + (ey*ey) )
        int_e+=e_diag
        Vel = ( 0.1*e_diag )*Flag_r
        Omega = ( 2*np.arctan2( np.sin(e_heading), np.cos(e_heading) ) )*Flag_r

        # Get the required velocity of each wheel to reach the goal
        self.l_w_msg = (1/(2*R))*(2*Vel-Omega*L)
        self.r_w_msg = (1/(2*R))*(2*Vel+Omega*L)

        # If ArUco marker is detected, reach the marker and wait for the confirmation
        if state=='M_detected':
            # If the button have not been pressed and the total error is 0, the goal
            # have been reached, so the program waits for the confirmation button
            if Button==0:
                if e_t==0.08:
                    Button = input('Enter 1: ')
                    my_list.clear()
            elif Button==1:
            # If the button have been pressed and the total error is 0, the robot
            # have been arrived to the initial position, then waits for a new table
            # request
                if e_t==0.0:
                    Button = 0
                    aruco_num = input('Enter ArUco Marker id: ')
        # If no ArUco marker is detected, set the designed path points as the goal
        if state=='No_M_detected':
            if e_t<=0.2:
                if i<59:
                    i+=1
                else:
                    i=59
        # Publish the messages to the topics
        self.left_wheel_vel.publish(self.l_w_msg)
        self.right_wheel_vel.publish(self.r_w_msg)
        self.rate.sleep()


    def cam_callback(self, data):
        global state
        global id_to_find
        try:
            # Convert the image message from the simulated camera to cv image
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            ## This part is for the real camera ##
            # ret, frame=cap.read()
            # gray=cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            # cv.imshow('Image', frame)
            # aruco_image = frame

            aruco_image = cv_image
            id_to_find = aruco_num
            marker_size = 15 #-[cm]
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
            # Convert in gray scale
            gray=cv.cvtColor(aruco_image, cv.COLOR_BGR2GRAY)

            # FInd all the aruco markers in the image
            corners, ids, rejected=aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
            if ids is not None and ids[0] == id_to_find:
                state = 'M_detected'
                aruco_image = aruco.drawDetectedMarkers(aruco_image, corners)
                ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
                try:
                    rvec, tvec=ret[0][0,0,:], ret[1][0,0,:]
                    aruco.drawAxis(aruco_image, camera_matrix, camera_distortion, rvec, tvec, 10)
                    str_position= 'Marker Postion x=%4.0f  y=%4.0f  z=%4.0f'%(tvec[0], tvec[1], tvec[2])
                    cv.putText(aruco_image, str_position, (0,290), font, 1, (0, 255, 0), 1, cv.LINE_AA)
                    #-- Obtain the rotation matrix tag->camera
                    R_ct    = np.matrix(cv.Rodrigues(rvec)[0])
                    R_tc    = R_ct.T
                    #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
                    roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)
                    #-- Print the marker's attitude respect to camera frame
                    str_attitude = "Marker Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                        math.degrees(yaw_marker))
                    cv.putText(aruco_image, str_attitude, (0, 315), font, 1, (0, 255, 0), 1, cv.LINE_AA)

                    # #-- Now get Position and attitude f the camera respect to the marker
                    # pos_camera = -R_tc*np.matrix(tvec).T
                    #
                    # str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
                    # cv.putText(aruco_image, str_position, (0, 340), font, 1, (255, 255, 0), 1, cv.LINE_AA)
                    #
                    # #-- Get the attitude of the camera respect to the frame
                    # roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
                    # str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                    #                     math.degrees(yaw_camera))
                    # cv.putText(aruco_image, str_attitude, (0, 365), font, 1, (255, 255, 0), 1, cv.LINE_AA)
                    # cv.putText(aruco_image, str(aruco_num) , (0, 380), font, 1, (0, 0, 255), 1, cv.LINE_AA)

                except:
                    print('No ArUco Detected')
            # else:
            #     state = 'No_M_detected'
            # Display the frame
            cv.imshow('frame', aruco_image)
            #cv.imwrite('/home/andresvergara/images_aruco/pics/img4.jpg', cv_image)
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
        # Get the distance reading from the sensor
        dist1 = round(msg_ls1.ranges[359], 2)
        phi1 = (np.pi/2)+theta
        # If the distance is less than 30 cm, switch to the obstacle avoidance behavior
        if dist1<0.3:
            # Control signals:
            # Vel = ( 0.05*np.sqrt( (ex*ex) + (ey*ey) ) )
            # Omega= ( 100*-np.arctan2( np.sin(e_heading), np.cos(e_heading) ) )

            self.l_w_msg = 1
            self.r_w_msg = 0
            Flag1 = 0
        else:
            Flag1 = 1
        self.left_wheel_vel.publish(self.l_w_msg)
        self.right_wheel_vel.publish(self.r_w_msg)
    def ls2_callback(self, msg_ls2):
        global Flag2
        dist2 = round(msg_ls2.ranges[359], 2)
        phi2 = -(np.pi/2)+theta
        if dist2<0.3:
            # Control signals:
            Vel = ( 0.05*np.sqrt( (ex*ex) + (ey*ey) ) )
            Omega= ( 100*-np.arctan2( np.sin(e_heading), np.cos(e_heading) ) )

            self.l_w_msg = (1/(2*R))*(2*Vel-Omega*L)
            self.r_w_msg = (1/(2*R))*(2*Vel+Omega*L)
            Flag2 = 0
        else:
            Flag2 = 1
        self.left_wheel_vel.publish(self.l_w_msg)
        self.right_wheel_vel.publish(self.r_w_msg)
    def ls3_callback(self, msg_ls3):
        global Flag3
        # Get the distance reading from the sensor
        dist3 = round(msg_ls3.ranges[359], 2)
        phi3 = (3*np.pi/4)+theta
        # If the distance is less than 30 cm, switch to the obstacle avoidance behavior
        if dist3<0.3:
            # Control signals:
            Vel = ( 0.05*np.sqrt( (ex*ex) + (ey*ey) ) )
            Omega= ( 100*-np.arctan2( np.sin(e_heading), np.cos(e_heading) ) )

            self.l_w_msg = (1/(2*R))*(2*Vel-Omega*L)
            self.r_w_msg = (1/(2*R))*(2*Vel+Omega*L)
            Flag3 = 0
        else:
            Flag3 = 1
        self.left_wheel_vel.publish(self.l_w_msg)
        self.right_wheel_vel.publish(self.r_w_msg)
    def ls4_callback(self, msg_ls4):
        global Flag4
        dist4 = round(msg_ls4.ranges[359], 2)
        phi4 = -(3*np.pi/4)+theta
        if dist4<0.3:
            # Control signals:
            Vel = ( 0.05*np.sqrt( (ex*ex) + (ey*ey) ) )
            Omega= ( 100*-np.arctan2( np.sin(e_heading), np.cos(e_heading) ) )

            self.l_w_msg = (1/(2*R))*(2*Vel-Omega*L)
            self.r_w_msg = (1/(2*R))*(2*Vel+Omega*L)
            Flag4 = 0
        else:
            Flag4 = 1
        self.left_wheel_vel.publish(self.l_w_msg)
        self.right_wheel_vel.publish(self.r_w_msg)




if __name__ == '__main__':
    # Number of the table request:
    aruco_num = input('Enter ArUco Marker id: ')
    Flag1 = 0
    Flag2 = 0
    Flag3 = 0
    Flag4 = 0
    Button = 0
    int_e = 0
    state = 'No_M_detected'
    i = 0
    R = 0.065
    L = 0.3
    global smallest
    smallest = None

    my_list = list()
    cord_x=list()
    cord_y=list()

    # Path to follow when no ArUco marker is detected:
    my_listx = list()
    my_listy = list()
    tray_x=np.linspace(1.0, 1.1, num=10)
    tray_y=np.linspace(6.0, 6.0, num=10)
    tray_x2=np.linspace(1.1, 1.1, num=10)
    tray_y2=np.linspace(6.0, -7.25, num=10)
    tray_x3=np.linspace(1.1, 1.0, num=10)
    tray_y3=np.linspace(-7.25, -7.25, num=10)
    tray_x4=np.linspace(1.0, 1.0, num=10)
    tray_y4=np.linspace(-7.25, 0, num=10)
    tray_x5=np.linspace(1.0, -1.5, num=10)
    tray_y5=np.linspace(0, 0, num=10)
    tray_x6=np.linspace(-1.5, -1.5, num=10)
    tray_y6=np.linspace(0, -7.25, num=10)
    for n in range(0,10):
        my_listx.append(tray_x[n])
    for n1 in range(0,10):
        my_listx.append(tray_x2[n1])
    for n2 in range(0,10):
        my_listx.append(tray_x3[n2])
    for n3 in range(0,10):
        my_listx.append(tray_x4[n3])
    for n4 in range(0,10):
        my_listx.append(tray_x5[n4])
    for n5 in range(0,10):
        my_listx.append(tray_x6[n5])
    for m in range(0,10):
        my_listy.append(tray_y[m])
    for m1 in range(0,10):
        my_listy.append(tray_y2[m1])
    for m2 in range(0,10):
        my_listy.append(tray_y3[m2])
    for m3 in range(0,10):
        my_listy.append(tray_y4[m3])
    for m4 in range(0,10):
        my_listy.append(tray_y5[m4])
    for m5 in range(0,10):
        my_listy.append(tray_y6[m5])

    ## This part is for the real camera ##
    # cap=cv.VideoCapture(0)
    # cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
    # cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

    kt = Controller()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
            plt.plot(cord_x,cord_y, 'g')
            plt.plot(1,6, 'x')
            plt.plot(2.4,-2, 'o')
            plt.plot(2.4,-5.5, 'o')
            plt.plot(2.4,-9, 'o')
            plt.plot(-.2,-2, 'o')
            plt.plot(-.2,-5.5, 'o')
            plt.plot(-.2,-9, 'o')
            plt.plot(-2.8,-2, 'o')
            plt.plot(-2.8,-5.5, 'o')
            plt.plot(-2.8,-9, 'o')
            plt.plot(tray_x6[9],tray_y6[9], 'D')
            plt.plot(my_listx,my_listy, 'b')
            plt.show()

    except rospy.ROSInterruptException as e:
        print(e)
