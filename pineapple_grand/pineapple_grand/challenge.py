import math
import time

import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from std_msgs.msg import String
import json 
from typing import List
from geometry_msgs.msg import Twist


class GrandChallenge(Node):

    """Grand Challenge: Create Action Client, Check if the box is detected, Navigate according to the weight; use movelt to grab, navigate to the shelf, repeat."""

    # Code from hs_pick_place.py (week-11)
    JOINT_NAMES: List[str] = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
    PLANNING_GROUP: str = 'tmr_arm'
    MOVELT_ACTION_NAME: str = '/move_action'  # Changed this to the Movelt Action
    NAV_ACTION_NAME: str = "/navigate_to_pose" # Changed this to the Nav 2 Action

    # Initializing the Class    
    def __init__(self):
        super().__init__('grand_challenge')

        # Defining the Movelt and Nav 2 Action Client
        self.movelt_client = ActionClient(self, MoveGroup, self.MOVELT_ACTION_NAME)
        self.navigate_client = ActionClient(self,NavigateToPose,self.NAV_ACTION_NAME)
        self.get_logger().info('Waiting for MoveGroup & NavigateToPose action server...') # Printing the message
        self.movelt_client.wait_for_server() 
        self.get_logger().info('MoveGroup action server ready.')
        self.navigate_client.wait_for_server()
        self.get_logger().info('NavigateToPose action server ready')

        # Subscribing to the /hmi/unified_status (Code from plc_hmi_listener.py)
        self.subscription = self.create_subscription(String,'/hmi/unified_status',self.listener_callback,10)
        
        # Publishing velocity commands to the /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Creating instances to store the box information
        self.box = None
        self.hmi_triggered = False
        self.last_count = None 
        self.weight = None
    
    # --- Helper function to build a PoseStamped (Code from hs_waypoint_follower.py) ---
    @staticmethod 
    def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
        """
        Create a PoseStamped (position + orientation) in the 'map' frame.
        - x, y are coordinates in meters
        - yaw is robot orientation (heading) in radians
        """
        ps = PoseStamped()
        ps.header.frame_id = 'map'  # always use 'map' for navigation goals
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)

        # Convert yaw (in radians) into quaternion (needed by ROS2)
        half = yaw * 0.5
        ps.pose.orientation.z = math.sin(half)
        ps.pose.orientation.w = math.cos(half)
        return ps

    # Code from hs_pick_place.py
    def _goal_from_joints(self, joints: List[float]) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = self.PLANNING_GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

        cs = Constraints()
        for name, val in zip(self.JOINT_NAMES, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            cs.joint_constraints.append(jc)

        req.goal_constraints.append(cs)
        goal.request = req
        goal.planning_options.plan_only = False  # plan + execute
        return goal

    # Code from hs_pick_place.py (Changed the method's name)
    def _send_arm_and_wait(self, joints: List[float]) -> bool:
        goal = self._goal_from_joints(joints)
        send_fut = self.movelt_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        handle = send_fut.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Goal rejected.')
            return False
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result()
        return bool(res and res.status == 4)  # 4 = STATUS_SUCCEEDED
    
    # Code from hs_pick_place.py (Changed the method's name)
    def send_nav_and_wait(self,pose: PoseStamped) -> bool:
        # Update timestamp (required in headers)

        pose.header.stamp = self.get_clock().now().to_msg()

        # Wrap pose in a NavigateToPose goal message
        goal = NavigateToPose.Goal()
        goal.pose = pose

        # Simple feedback callback: prints distance left to target
        def feedback_cb(fb):
            try:
                dist = fb.feedback.distance_remaining
                #self.get_logger().info(f'Distance remaining: {dist:.2f} m')
            except Exception:
                pass  # ignore if feedback doesn't have distance

        # Send the goal
        send_future = self.navigate_client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()

        if not handle or not handle.accepted:
            self.get_logger().error('Goal was rejected!')
            return False

        # Wait until navigation is finished
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            self.get_logger().error('No result returned.')
            return False

        # A finished goal is not always a successful one - check the status
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Goal failed (status {result.status}, '
                                    f'error code {result.result.error_code})')
            return False

        self.get_logger().info('Goal reached successfully!')
        return True

    # Callback function for the subscriber (Refrence taken from plc_hmi_listener.py)
    def listener_callback(self, msg):

        try:
            data = json.loads(msg.data)
            box = data["box"]
            counts = data["counts"]
            raw_weight = box.get("weight_raw")

            # Checking if the weight is empty 
            if raw_weight is None or not str(raw_weight).strip():
                return

            # Storing the weight as string (The value usually has ": num kg")
            weight_str = str(raw_weight).replace(":", "").replace("kg", "").strip()

            # Chaing it to int
            try:
                weight = int(float(weight_str))
            except ValueError:
                return
 
            # Keeping the weight to be 0 instead of null
            if self.weight is None:
                self.weight =0
            
            # Creates the instance for weight
            self.weight = weight
            
            # Creates the instance for total_counts
            total_count = counts["total"]
            
            # Checking if the instance is null or not
            if self.last_count is None:
                self.last_count = total_count # Stores the current count
                return
            
            # Checking if nex box has been added/ spawned
            if total_count > self.last_count:
                self.hmi_triggered = True
                self.get_logger().info("New Box Detcted!")

            # Storing the count
            self.last_count = total_count

        except Exception as e:
            self.get_logger().error(f"Failed to parse JSON: {e}\nRaw msg={msg.data}")

    
    # Publishser method, which makes the omron to reverse
    def reverse_robot(self):
        cmd = Twist() # Creates an object of Twist
        cmd.linear.x = -1.0  # Reverse at -1 m/s
        cmd.angular.z = 0.0  # No rotation

        # Publishes the commands 20 times at 0.1 seconds interval; so 0.1*20 = Reverses for 2 seconds 
        for i in range(20):
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)

        # Stop the reversing
        self.cmd_pub.publish(Twist())
    
    # Run method
    def run(self):

        # Position taken from Assignment 3.2   
        pick_position = [math.radians(2.0), math.radians(35.0), math.radians(90.0), math.radians(316.0),math.radians(86.0), math.radians(69.0)]
        carry_postion = [math.radians(2.0), math.radians(8.0), math.radians(70.0), math.radians(316.0), math.radians(86.0), math.radians(69.0)]
        place_position = [math.radians(2.0), math.radians(3.0), math.radians(110.0), math.radians(316.0),math.radians(86.0), math.radians(69.0)]

        # Used publish point to approximate the pick up location
        location_C = self.make_pose(2.83,0.7426536083221436,0.000)
        location_B = self.make_pose(2.82,-0.0332,0.000)
        location_A = self.make_pose(2.5,-0.65,0.000)

        # Used publish point tool in rviz to approximate the position
        shelf_position = self.make_pose(30.9,-4.72,-1.5714)
        try:
            while rclpy.ok():
                while rclpy.ok() and not self.hmi_triggered:
                    rclpy.spin_once(self, timeout_sec=0.1)

                if not rclpy.ok():
                    break
                
                self.hmi_triggered = False

                # Condition for small box
                if self.weight>0 and self.weight<=5:
                    self.get_logger().info('Navigating to location C')
                    if not self.send_nav_and_wait(location_C):
                        return

                    self.get_logger().info("Pickin up the box")
                    if not self._send_arm_and_wait(pick_position):
                        return 

                    self.get_logger().info("Carrying the box")
                    if not self._send_arm_and_wait(carry_postion):
                        return

                    self.carry_postion = True
                    self.get_logger().info("Turnign Around")    
                    self.reverse_robot()

                    if self.carry_postion == True:
                        self.get_logger().info("Putting the box in shelf")
                        if not self.send_nav_and_wait(shelf_position):
                            return

                        self.get_logger().info("Placing the box")
                        if not self._send_arm_and_wait(place_position):
                            return 

                        self.carry_postion = False
                        
                        self.get_logger().info("Turning Around")
                        self.reverse_robot()
                    
                    self.get_logger().info("Action Completed. Please spawn a new one")
                
                # Condition for medium box
                if self.weight>5 and self.weight<=10:
                    self.get_logger().info('Navigating to location B')
                    if not self.send_nav_and_wait(location_B):
                        return
                    
                    self.get_logger().info("Pickin up the box")
                    if not self._send_arm_and_wait(pick_position):
                        return 

                    self.get_logger().info("Carrying the box")
                    if not self._send_arm_and_wait(carry_postion):
                        return

                    self.carry_postion = True
                    self.get_logger().info("Turning Around")
                    self.reverse_robot()
                    
                    if self.carry_postion == True:
                        self.get_logger().info("Putting the box in shelf")
                        if not self.send_nav_and_wait(shelf_position):
                            return
                        
                        self.get_logger().info("Placing the box")
                        if not self._send_arm_and_wait(place_position):
                            return

                        self.carry_postion = False

                        self.get_logger().info("Turning Around")
                        self.reverse_robot()
                    
                    self.get_logger().info("Action completed. Please spawn a new one")
                    
                if self.weight>10:
                    self.get_logger().info('Navigating to location A')
                    if not self.send_nav_and_wait(location_A):
                        return 

                    self.get_logger().info("Pickin up the box")
                    if not self._send_arm_and_wait(pick_position):
                        return 

                    self.get_logger().info("Carrying the box")
                    if not self._send_arm_and_wait(carry_postion):
                        return
                    
                    self.carry_postion = True
                    self.get_logger().info("Turning Around")
                    self.reverse_robot()

                    if self.carry_postion == True:
                        self.get_logger().info("Putting the box in shelf")
                        
                        if not self.send_nav_and_wait(shelf_position):
                            return

                        self.get_logger().info("Placing the box")
                        if not self._send_arm_and_wait(place_position):
                            return


                        self.carry_postion = False
                        self.get_logger().info("Turning Around")
                        self.reverse_robot()
                    
                    self.get_logger().info("Action completed. Please spawn a new one")
                    

        except (json.JSONDecodeError, KeyError, TypeError, AttributeError) as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")



# Main fucntion that runs the node
def main(args=None):
    rclpy.init(args=args)
    node = GrandChallenge()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()