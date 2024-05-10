#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

help_msg = """

Control command and key board!
---------------------------
enable  save: s
disable save: d
"""

loop_rate = 10

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def m_publisher():
    pub = rospy.Publisher('key_checker', String, queue_size=10)
    rospy.init_node('key_checker_node', anonymous=True)
    rate = rospy.Rate(loop_rate)
    i=0

    print("This node is working good!!")
    print("Press [S] = Robot is start from start zone!!!")
    print("      [R] = Robot is start from retry zone!!!")

    while not rospy.is_shutdown():
        key = getKey()
        if key == 's' :
            os.system('clear')
            print("Press [S] = Robot is start from start zone!!!\r")
            print("      [R] = Robot is start from retry zone!!!\r")
            rospy.logwarn("Robot is starting\r")
            rospy.loginfo("Game status = 2\r")
            pub.publish(key)
        elif key == 'r' :
            os.system('clear')
            print("Press [S] = Robot is start from start zone!!!\r")
            print("      [R] = Robot is start from retry zone!!!\r")
            rospy.logerr("Robot retrying!!!\r")
            rospy.logwarn("Game status = 4\r")
            pub.publish(key)
        else:
            if (key == '\x03'):
                rospy.loginfo(key)
                print('\r')
                rospy.logerr("key_checker node is ended!!!!\r")
                os.system('clear')
                break
        rate.sleep()

if __name__=='__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    print("key_checker node is running...\r")
    try:
        m_publisher()
    except rospy.ROSInterruptException:
        print("into exception")
        pass

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)