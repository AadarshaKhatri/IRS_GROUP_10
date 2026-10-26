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


class GrandChallenge(Node):

    JOINT_NAMES: List[str] = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
    PLANNING_GROUP: str = 'tmr_arm'
    MOVELT_ACTION_NAME: str = '/move_action'  # keep identical to your setup
    NAV_ACTION_NAME: str = "/navigate_to_pose"

    def __init__(self):
        super().__init__('hs_waypoint_runner')
        self.movelt_client = ActionClient(self, MoveGroup, self.MOVELT_ACTION_NAME)
        self.navigate_client = ActionClient(self,NavigateToPose,self.NAV_ACTION_NAME)
        self.get_logger().info('Waiting for MoveGroup & NavigateToPose action server...')
        self.movelt_client.wait_for_server()
        self.get_logger().info('MoveGroup action server ready.')
        self.navigate_client.wait_for_server()
        self.get_logger().info('NavigateToPose action server ready')

        self.subscription = self.create_subscription(String,'/hmi/unified_status',self.listener_callback,10)
        
        self.box = None
        self.hmi_triggered = False
        self.last_count = None 
        self.location = None
        self.currently_attached = False

    # --- Helper function to build a PoseStamped ---
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

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)

            box = data["box"]
            counts = data["counts"]

            location = str(box.get("location", "")).replace(":", "").strip()
            total_count = counts.get("total")

            if location not in ["A", "B", "C"]:
                return

            if total_count is None:
                return

            total_count = int(total_count)

            # First message: initialise count without navigating
            if self.last_count is None:
                self.last_count = total_count
                return

            # Trigger only when a NEW box has reached its location
            if total_count > self.last_count:
                self.location = location
                self.hmi_triggered = True

                self.get_logger().info(
                    f"New box at location {location}, total count: {total_count}"
                )

            self.last_count = total_count

        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as e:
            self.get_logger().error(f"HMI error: {e}")
    
    def run(self):

        original_position = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]
        pick_position = [math.radians(2.0), math.radians(35.0), math.radians(90.0), math.radians(316.0),math.radians(86.0), math.radians(69.0)]
        carry_postion = [math.radians(2.0), math.radians(8.0), math.radians(70.0), math.radians(316.0), math.radians(86.0), math.radians(69.0)]
        place_position = [math.radians(2.0), math.radians(3.0), math.radians(110.0), math.radians(316.0),math.radians(86.0), math.radians(69.0)]

        location_C = self.make_pose(2.99,1,0)
        location_B = self.make_pose(3.09,0.00391,0)
        location_A = self.make_pose(2.99, -1.01,0)

        turn_around_from_conveyor = self.make_pose(1,1,3.14)
        turn_around_from_shelf = self.make_pose(30.9,-4.72,1.5714)
        
        shelf_position = self.make_pose(30.9,-4.72,-1.5714)
        try:
            while rclpy.ok():
                while rclpy.ok() and not self.hmi_triggered:
                    rclpy.spin_once(self, timeout_sec=0.1)

                if not rclpy.ok():
                    break
                
                self.hmi_triggered = False
                time.sleep(7.0)
                if self.location == 'C':
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
                    if not self.send_nav_and_wait(turn_around_from_conveyor):
                        return 

                    if self.carry_postion == True:
                        self.get_logger().info("Putting the box in shelf")
                        if not self.send_nav_and_wait(shelf_position):
                            return

                        self.get_logger().info("Placing the box")
                        if not self._send_arm_and_wait(place_position):
                            return 

                        self.carry_postion = False
                        
                        self.get_logger().info("Turning Around")
                        if not self.send_nav_and_wait(turn_around_from_shelf):
                            return 
                    
                    self.get_logger().info("Placed the box at the shelf.")
                    
                if self.location == 'B':
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
                    if not self.send_nav_and_wait(turn_around_from_conveyor):
                        return 
                    
                    if self.carry_postion == True:
                        self.get_logger().info("Putting the box in shelf")
                        if not self.send_nav_and_wait(shelf_position):
                            return
                        
                        self.get_logger().info("Placing the box")
                        if not self._send_arm_and_wait(place_position):
                            return

                        self.carry_postion = False

                        self.get_logger().info("Turning Around")
                        if not self.send_nav_and_wait(turn_around_from_shelf):
                            return 
                    
                    self.get_logger().info("Placed the box at the shelf.")
                    
                if self.location == 'A':
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
                    if not self.send_nav_and_wait(turn_around_from_conveyor):
                        return

                    if self.carry_postion == True:
                        self.get_logger().info("Putting the box in shelf")
                        
                        if not self.send_nav_and_wait(shelf_position):
                            return

                        self.get_logger().info("Placing the box")
                        if not self._send_arm_and_wait(place_position):
                            return


                        self.carry_postion = False

                        self.get_logger("Turning Around")
                        if not self.send_nav_and_wait(turn_around_from_shelf):
                            return 
                    
                    self.get_logger().info("Placed the box at the shelf.")
                    

        except (json.JSONDecodeError, KeyError, TypeError, AttributeError) as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")


        # Waypoint 1
        if not self._send_and_wait([0.0, 0.0, 1.57, 0.0, 1.57, 0.0]): return
        time.sleep(10.0)


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