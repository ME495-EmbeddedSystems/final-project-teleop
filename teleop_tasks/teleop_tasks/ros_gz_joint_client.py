import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState

# Helpper functions
def createJointNamedict():
    jointDict = {"palm_x":0,
                 "palm_y":1,
                 "palm_z":2,
                 "palm_roll":3,
                 "palm_pitch":4,
                 "palm_yaw":5,
                 "palm_thumb_dumb":6,
                 "thumb_dumb_thumb_1":7,
                 "thumb_1_thumb_2":8,
                 "palm_index_dumb":9,
                 "index_dumb_index_1":10,
                 "index_1_index_2":11,
                 "index_2_index_3":12,
                 "palm_middle_dumb":13,
                 "middle_dumb_middle_1":14,
                 "middle_1_middle_2":15,
                 "middle_2_middle_3":16,
                 "palm_ring_dumb":17,
                 "ring_dumb_ring_1":18,
                 "ring_1_ring_2":19,
                 "ring_2_ring_3":20,
                 "palm_pinky_dumb":21,
                 "pinky_dumb_pinky_1":22, 
                 "pinky_1_pinky_2":23,
                 "pinky_2_pinky_3":24}
    
    return jointDict

class Ros_gz_joint_client(Node):
    def __init__(self):
        super().__init__("ros_gz_joint_client")
        # Subscriber to joint trajectories
        self.timer = self.create_timer(1/100, self.timerCallback)
        
        # Create joint states subscriber
        self.haptx_sub = self.create_subscription(JointState, "/left_hand/joint_states",
                                                  self.jointStateListener, 10)
        
        # Action client
        self.ControllerAvailable = False
        self.callbackGroup = MutuallyExclusiveCallbackGroup()
        self.Mover = ActionClient(self, FollowJointTrajectory,
                                  "/joint_trajectory_controller/follow_joint_trajectory",
                                  callback_group=self.callbackGroup)
        
        if not self.Mover.wait_for_server(timeout_sec=3.0):
            errorMSG = RuntimeError("Controller not available")
            self.get_logger().debug("Controller not available")
            self.get_logger().debug(f"{errorMSG}")
        else:
            self.ControllerAvailable = True
            
        self.goalSent = False
        self.goal_handle = 0
        
        # Joint names dictionary
        self.jointDict = createJointNamedict()
        
        # Creating a message now that will get sent each time step
        # Create message
        self.message = FollowJointTrajectory.Goal()
        
        self.message.goal_time_tolerance = Duration(seconds=1.0).to_msg() # Start imediately
        
        self.message.trajectory.joint_names = ["palm_x", "palm_y", "palm_z", "palm_roll",
                                               "palm_pitch", "palm_yaw", "palm_thumb_dumb",
                                               "thumb_dumb_thumb_1", "thumb_1_thumb_2",
                                               "palm_index_dumb", "index_dumb_index_1",
                                               "index_1_index_2", "index_2_index_3",
                                               "palm_middle_dumb", "middle_dumb_middle_1",
                                               "middle_1_middle_2", "middle_2_middle_3",
                                               "palm_ring_dumb", "ring_dumb_ring_1",
                                               "ring_1_ring_2", "ring_2_ring_3",
                                               "palm_pinky_dumb", "pinky_dumb_pinky_1", 
                                               "pinky_1_pinky_2", "pinky_2_pinky_3"]
        
        # Creating the path
        self.point1 = JointTrajectoryPoint()
        self.point1.time_from_start = Duration(seconds=0.0).to_msg()
        self.point1.positions = [0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0,]
        self.point2 = JointTrajectoryPoint()
        self.point2.time_from_start = Duration(seconds=0.1).to_msg()
        self.point2.positions = [0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0,]
        
    def jointStateListener(self, msg=JointState):
        # When a new joint state is sent to the topic make 
        # point1 = point2
        # point2 = newPoints
        self.point1.positions = self.point2.positions
        
        for i in len(msg.name):
            # Map the names of the joints in the message to the
            # positions in point2
            self.point2.positions[self.jointDict[msg.name[i]]] = msg.position[i]
    
    async def timerCallback(self):
        if self.goalSent == False:
            self.message.trajectory.points = [self.point1, self.point2]
            
            # Send action call
            self.goal_handle = await self.Mover.send_goal_async(self.message)
            
        else:
            if not self.goal_handle.accepted:
                self.get_logger().info("Goal not accepted")
                return
            self.get_logger().info("Goal accepted.")

            res = await self.goal_handle.get_result_async()
            result = res.result
            status = res.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                print("Goal succeeded!")
            else:
                print("Goal failed with status: {0}".format(status))
                
            print(result)
            print(status)

def main(args=None):
    """Objects' main function."""
    rclpy.init(args=args)
    node = Ros_gz_joint_client()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
