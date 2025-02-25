import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = False

    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp


    # Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    #       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
    # Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
    def extract_vehicle_info(self, currentPose):

        ####################### TODO: Your TASK 1 code starts Here #######################
        pos_x, pos_y, vel, yaw = 0, 0, 0, 0
        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y

        # Calculate velocity as the Euclidean norm of the linear velocity in x and y
        vel_x = currentPose.twist.linear.x
        vel_y = currentPose.twist.linear.y
        vel = (vel_x**2 + vel_y**2) ** 0.5

        # Convert quaternion to Euler angles using the helper function, then extract yaw (in radians)
        # Assume quaternion_to_euler returns (roll, pitch, yaw)
        _, _, yaw = quaternion_to_euler(currentPose.pose.orientation.x,
                                            currentPose.pose.orientation.y,
                                            currentPose.pose.orientation.z,
                                            currentPose.pose.orientation.w)
        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################
        target_velocity = 10
        num_points=len(future_unreached_waypoints)
        bounds = [0.4,0.7]
        if num_points>5:
            recent_points=future_unreached_waypoints[0:5]
        else:
            recent_points=future_unreached_waypoints
        speeds =[]
        for i,wp in enumerate(recent_points):
            wp_x, wp_y = wp
        
            # Compute the desired heading from the current position to the waypoint
            desired_yaw = math.atan2(wp_x-curr_x , wp_y-curr_y)
            
            # Normalize the difference to the range [0, pi]
            angle_diff = abs((desired_yaw - curr_yaw + math.pi) % (2 * math.pi) - math.pi)
            
            # Mapping: 
            # For angle_diff < 0.1 rad, target speed = 12 m/s (straight path)
            # For angle_diff > 0.5 rad, target speed = 8 m/s (sharp turn)
            # For values in between, linearly interpolate between 12 and 8 m/s.
            if angle_diff < bounds[0]:
                target_speed = 12.0
            elif angle_diff > bounds[1]:
                target_speed = 8.0
            else:
                target_speed = (4 - num_points*((angle_diff - bounds[0]) * ((bounds[1] - bounds[0]))/i))+8
            
            speeds.append(target_speed)
    
    # Choose the minimum speed from all evaluated waypoints to ensure safety.
        target_velocity = min(speeds)
        ####################### TODO: Your TASK 2 code ends Here #######################
        return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y,curr_vel, curr_yaw, target_point, future_unreached_waypoints):

        ####################### TODO: Your TASK 3 code starts Here #######################
        target_index = 0
        target_steering = 0
        K_dd = 0.8
        min_ld, max_ld = 5, 30
        l_d_min = np.clip(K_dd * curr_vel, min_ld, max_ld)
        tar_x,tar_y = target_point
        l_d = np.linalg.norm(wp-[curr_x,curr_y])
        if np.linalg.norm(wp-[curr_x,curr_y])>l_d_min:
            target_yaw = math.atan2(tar_x-curr_x , tar_y-curr_y )
            alpha = math.abs(target_yaw-curr_yaw)
            target_steering = math.atan2((2*self.L*math.sin(alpha)/l_d))
            return target_steering
        for wp in future_unreached_waypoints:
            tar_x,tar_y = wp
            if np.linalg.norm(wp-[curr_x,curr_y])>l_d:
                target_yaw = math.atan2(tar_x-curr_x , tar_y-curr_y )
                alpha = math.abs(target_yaw-curr_yaw)
                target_steering = math.atan2((2*self.L*math.sin(alpha)/l_d))
                break
        ####################### TODO: Your TASK 3 code starts Here #######################
        return target_steering


    def execute(self, currentPose, target_point, future_unreached_waypoints):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)

        # Acceleration Profile
        if self.log_acceleration:
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz



        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y,curr_vel, curr_yaw, target_point, future_unreached_waypoints)


        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)
