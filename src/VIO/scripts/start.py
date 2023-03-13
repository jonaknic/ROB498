import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped, Pose
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import signal
import time
import math


class challenge2:
    def __init__(self):
        self.current_state = State()
        self.node_name = 'rob498_drone_XX'
        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(20)

        self.srv_launch = rospy.Service(self.node_name + '/comm/launch', Empty, self.callback_launch)
        self.srv_test = rospy.Service(self.node_name + '/comm/test', Empty, self.callback_test)
        self.srv_land = rospy.Service(self.node_name + '/comm/land', Empty, self.callback_land)
        self.srv_abort = rospy.Service(self.node_name + '/comm/abort', Empty, self.callback_abort)

        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = self.print_pose)

        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.arm_cmd = CommandBoolRequest()

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.launch_proxy = rospy.ServiceProxy(self.node_name + '/comm/launch', Empty)
        self.test_proxy = rospy.ServiceProxy(self.node_name + '/comm/test', Empty)
        self.land_proxy = rospy.ServiceProxy(self.node_name + '/comm/land', Empty)
        self.abort_proxy = rospy.ServiceProxy(self.node_name + '/comm/abort', Empty)

    # Callback handlers
    def handle_launch(self):

        print('Launch Requested. Your drone should take off.')
        print("[    ]")
        #rospy.wait_for_service("/mavros/cmd/arming")

        self.arm_cmd.value = True
        self.arming_client.call(self.arm_cmd)

        launch_set_mode = SetModeRequest()
        launch_set_mode.custom_mode = "OFFBOARD"

        goal_pose = PoseStamped()
        goal_pose.pose.position.x = 0
        goal_pose.pose.position.y = 0
        goal_pose.pose.position.z = 0.3

        self.set_mode_client.call(launch_set_mode)
        if self.current_state.mode == "OFFBOARD":
            rospy.loginfo("OFFBOARD enabled")
        err=self.norm_dist(goal_pose)
        while err > 0.1:
            rospy.loginfo(self.current_state.mode)
            err=self.norm_dist(goal_pose)
            print(f"Error: {err}")
            self.local_pos_pub.publish(goal_pose)
            self.rate.sleep()
        rospy.loginfo("REACHED ")

        while(not rospy.is_shutdown()):
            launch_set_mode = SetModeRequest()
            launch_set_mode.custom_mode = "AUTO.LAND"
            self.set_mode_client.call(launch_set_mode)
            rospy.loginfo(self.current_state.mode)

    def handle_test(self):
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
        print("[    ]")
        # TODO

    def handle_land(self):
        print('Land Requested. Your drone should land.')
        print("[    ]")
        self.arm_cmd.value = True
        self.arming_client.call(self.arm_cmd)

        launch_set_mode = SetModeRequest()
        launch_set_mode.custom_mode = "POSITION"

        goal_pose = PoseStamped()
        goal_pose.pose.position.x = self.cur_pose.pose.position.x
        goal_pose.pose.position.y = self.cur_pose.pose.position.y
        goal_pose.pose.position.z = 0.05
        self.set_mode_client.call(launch_set_mode)

        if self.current_state.mode=="POSITION":
            rospy.loginfo("POSITION enabled")

        err=self.norm_dist(goal_pose)
        while err>0.05:
            err=self.norm_dist(goal_pose)
            print(f"Error: {err}")
            self.local_pos_pub.publish(goal_pose)
            self.rate.sleep()
        rospy.loginfo("REACHED")

        launch_set_mode = SetModeRequest()
        launch_set_mode.custom_mode = "STABILIZED"
        self.set_mode_client.call(launch_set_mode)
        rospy.loginfo(self.current_state.mode)

    def handle_abort(self):
        print('Abort Requested. Your drone should land immediately due to safety considerations')
        self.land_proxy()
        self.arm_cmd.value = False
        self.arming_client.call(self.arm_cmd)
        print("killing")

    def state_cb(self,msg):
        self.current_state
        self.current_state = msg

    def print_pose(self,msg):
        self.cur_pose=msg

    # Service callbacks
    def callback_launch(self,request):
        self.handle_launch()
        return EmptyResponse()

    def callback_test(self,request):
        self.handle_test()
        return EmptyResponse()

    def callback_land(self,request):
        self.handle_land()
        return EmptyResponse()

    def callback_abort(self,request):
        self.handle_abort()
        return EmptyResponse()

    # Main communication node for ground control
    def comm_node(self):
        print('This is a dummy drone node to test communication with the ground control')
        print('node_name should be rob498_drone_TeamID. Service topics should follow the name convention below')
        print('The TAs will test these service calls prior to flight')
        print('Your own code should be integrated into this node')

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.current_state.connected):
            self.rate.sleep()

        rospy.spin()

    def norm_dist(self,goal):
        dx=self.cur_pose.pose.position.x-goal.pose.position.x
        dy=self.cur_pose.pose.position.y-goal.pose.position.y
        dz=self.cur_pose.pose.position.z-goal.pose.position.z
        print(dx,dy,dz)
        return math.sqrt(dx**2+dy**2+dz**2)

if __name__ == "__main__":
    c2=challenge2()
    c2.comm_node()