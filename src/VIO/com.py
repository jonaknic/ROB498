import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped, Pose
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


node_name = 'rob498_drone_XX'
#srv_abort = rospy.Service(node_name + '/comm/abort', Empty, callback_abort)
launch_proxy = rospy.ServiceProxy(node_name + '/comm/launch', Empty)
test_proxy = rospy.ServiceProxy(node_name + '/comm/test', Empty)
land_proxy = rospy.ServiceProxy(node_name + '/comm/land', Empty)
abort_proxy = rospy.ServiceProxy(node_name + '/comm/abort', Empty)

if __name__ == "__main__":
    x='w'
    while x=='w':
        x=raw_input('what do?')
        if x=='l':
            launch_proxy()
            x='w'
        elif x=='t':
            test_proxy()
            x='w'
        elif x=='d':
            land_proxy()
            x='w'
        elif x=='a':
            abort_proxy()
            x='w'
        elif x=='o':
            print('Exiting')
            break
        else:
            print("Try l, t, d, or a")
            x='w' 