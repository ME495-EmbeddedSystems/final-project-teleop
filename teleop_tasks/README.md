# teleop_tasks

## Quick launch
To run the rings simulation run `ros2 launch teleop_tasks rings.launch.xml`  

## Launch file descriptions
**hand.launch.xml**  
Run: `ros2 launch teleop_tasks hand.launch.xml`  
Launches a left hand in Rviz with joint state gui launched to more easily view what each joint. Also launches *finger_joint_mapper.py* and *ros_gz_joint_client.py* so the joints can be visualized from the haptx joint state publisher.
  
**leftHandTest.launch.xml**  
Run: `ros2 launch teleop_tasks leftHandTest.launch.xml`  
Launches a left hand in Rviz with joint state gui.  

**rings.launch.xml**  
Run: `ros2 launch teleop_tasks rings.launch.xml`  
Launches **leftHandTest.launch.xml**, *finger_joint_mapper.py*, and *ros_gz_joint_client.py* to have a complete visualization of the left hand in Rviz and gazebo. No joint_state_gui is launched, the hand takes in joint states from topic `/left_hand/joint_states`

## Node list  
*finger_joint_mapper*  
Reads in joint values from `/haptx/lh/raw_joint_states`, the topic that the haptx gloves publish their raw joint states to. Then maps them to the joints defined in leftHand.urdf.xacro.  

*obejcts*  
Reads in data about the objects in the gazebo virtual environment and publishes them to `/objects` so other nodes can record the position of the objects in question.  

*ros_gz_joint_client*  
Reads in joints published to `/left_hand/joint_states` and sends them to the ros_controller nodes that are created in the **rings.launch.xml** launch file. 

*controller_manager*  
Default controller manager node created to manage the controller nodes launched in **rings.launch.xml**  

