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


while True:
    cmd_input = raw_input("Command:")
    if cmd_input == 'l':  # launch
        launch_proxy()
    elif cmd_input == 't':  # test
        test_proxy()
    elif cmd_input == 'd':  # land
        land_proxy()
    elif cmd_input == 'a':  # abort
        abort_proxy()
    elif cmd_input == 'q':  # quit
        print("Quitting")
        break
    else:
        ("l: launch, t: test, d: land, a: abort, q: quit")
