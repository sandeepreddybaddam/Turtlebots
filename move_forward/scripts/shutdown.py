import rospy
from geometry_msgs.msg import Twist

def shut_down():
    rospy.init_node('shut_down', anonymous=True)
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()
    move_cmd.linear.x = 0
    cmd_vel.publish(move_cmd)
    r.sleep(1)

if __name__ == '__main__':
    shut_down()
