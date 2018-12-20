#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math
from std_srvs.srv import Empty

x=0
y=0
yaw=0
velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
def poseCallback(pose_message):
    global x,y,yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta
#poseSub = rospy.Subscriber('/turtle1/pose', Pose, callback=poseCallback)

def rotate(rotation_speed, rotation_angle, clockwise):
    global yaw
    theta0=yaw
    velocity_message=Twist()
    #rotation_publisher  = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    angle_rotated = 0.0
    start = time.time()
    loop_rate = rospy.Rate(10)
    if clockwise:
        velocity_message.angular.z=-abs(rotation_speed)
    else:
        velocity_message.angular.z=abs(rotation_speed)
    while True:
        rospy.loginfo("Robot turning")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        angle_moved = (time.time()-start)*rotation_speed
        if (angle_moved>rotation_angle):
            print("rotated")
            break
    velocity_message.angular.z=0
    velocity_publisher.publish(velocity_message)

def convert_to_radians(angle):
    return abs(angle * (3.14/180))

def setAngle(desired_angle):
    
    angle_to_move =  desired_angle - yaw
    print(angle_to_move)
    if angle_to_move<0:
        clockwise =1
    else:
        clockwise=0
    rotate(convert_to_radians(30), angle_to_move, clockwise)

def movetoGoal(x_goal, y_goal):
    global x,y,yaw
    velocity_message = Twist()
    #velocity_publisher=rospy.Publisher('/turtle1/cmd_vel', Twist,queue_size=10)
    distance_tolerance=0.01
    while True:
        linear_displacement = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))
        angular_displacement = math.atan2((y_goal-y), (x_goal-x))-yaw
        velocity_message.linear.x=1.0*linear_displacement
        velocity_message.angular.z=4*angular_displacement
        velocity_publisher.publish(velocity_message)
        print("moving to goal")
        print(x,y)
        print(linear_displacement)
        print(angular_displacement)
        if (linear_displacement<distance_tolerance):
            print("reached")
            break
    velocity_message.linear.x=0
    velocity_message.angular.x=0

def move(speed, distance, isforward):
    global x,y,yaw
    x0 = x
    y0 = y
    print(x0,y0)
    velocity_message=Twist()
    #velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    distance_moved=0.0
    loop_rate = rospy.Rate(10)
    start=time.time()
    if isforward:
        velocity_message.linear.x=abs(speed)
        #print("robot moves forward")
    else:
        velocity_message.linear.x=-abs(speed)
        #print("robot moves backward")
    velocity_message.linear.y=0
    velocity_message.linear.z=0
    velocity_message.angular.x=0
    velocity_message.angular.y=0
    velocity_message.angular.z=0
    while True:
        rospy.loginfo("Turtlesim moves forwards")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        distance_moved = (time.time()-start)*speed
        #distance_moved = abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print(distance_moved)
        if not(distance_moved<distance):
            rospy.loginfo("reached")
            break
    velocity_message.linear.x=0
    velocity_publisher.publish(velocity_message)
def spiralCleaner():
    global x,y
    x0=x
    y0=y
    velocity_message = Twist()
    loop_rate = rospy.Rate(1)
    #current_pose = Pose()
    rk=1
    while(abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))<2):
        print(x, y)
        velocity_message.linear.x=rk+0.5
        velocity_message.linear.y=0
        velocity_message.linear.z=0
        velocity_message.angular.x=0
        velocity_message.angular.y=0
        velocity_message.angular.z=2.0
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        rk+=0.2
    velocity_message.linear.x=0
    velocity_message.angular.z=0
    velocity_publisher.publish(velocity_message)
def gridCleaner():
    movetoGoal(1.0,1.0)
    setAngle(convert_to_radians(0))

    move(2.0,4.0,1)
    rotate(convert_to_radians(30), convert_to_radians(90),0)
    move(2.0,4.0,1)
    rotate(convert_to_radians(30), convert_to_radians(90),0)
    move(2.0,1.0,1)
    rotate(convert_to_radians(30), convert_to_radians(90),0)
    move(2.0,4.0,1)
    rotate(convert_to_radians(30), convert_to_radians(90),1)
    move(2.0,1.0,1)
    pass

if __name__=='__main__':
    try:
        rospy.init_node('turtlesim_cleaner_robot')
        #velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        time.sleep(2)
        move(1.0,1.0,1)
        #rotate(convert_to_radians(30),convert_to_radians(90),1)
        #setAngle(convert_to_radians(90))
        #movetoGoal(3.0,6.0)
        #spiralCleaner()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
