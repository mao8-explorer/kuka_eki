import rospy
import tf
from geometry_msgs.msg import PointStamped

def tf_to_point():
    rospy.init_node('tf_to_point')
    tf_listener = tf.TransformListener()
    point_pub = rospy.Publisher('/tf_point', PointStamped, queue_size=10)
    rate = rospy.Rate(100.0)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = tf_listener.lookupTransform('/base_link', '/joint_a6_Link', rospy.Time(0))
            point = PointStamped()
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = '/base_link'
            point.point.x = trans[0]
            point.point.y = trans[1]
            point.point.z = trans[2]
            point_pub.publish(point)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

if __name__ == '__main__':
    try:
        tf_to_point()
    except rospy.ROSInterruptException:
        pass
