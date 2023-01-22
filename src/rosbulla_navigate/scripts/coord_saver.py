import rospy
import tf
import csv
import os
from std_msgs.msg import Int32

def callback(event):
    try:
        (trans,rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.loginfo('Failed %s', e)
        return
    path = os.path.expanduser('~/Documents/position_data.csv')
    with open(path, "a") as f:
        writer = csv.writer(f)
        writer.writerow([round(trans[0],2), round(trans[1],2), round(trans[2],2)])

def listener():
    rospy.init_node("coord_saver", anonymous=True)
    global listener
    listener = tf.TransformListener()
    rospy.Subscriber("coord_saver", Int32, callback)
    # rospy.Timer(rospy.Duration(0.1), callback)
    rospy.spin()

if __name__ == '__main__':
    listener()