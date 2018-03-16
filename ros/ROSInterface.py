import rospy, tf, dwl
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from dwl_msgs import WholeBodyStatePublisher as wb_pub


class ROSInterface():
    def __init__(self, fbs, rate, body_link):
        # Passing the floating-base system
        self.fbs = fbs

        self.pub_rate = rate
        self.last_pub_time = 0.
        self.base_link = body_link #fbs.getFloatingBaseBody()

        # Declaring the node
        rospy.init_node('dwl_csuite', anonymous=True)

        # Telling ROS master to used the simulation time for synchronization
        rospy.set_param('use_sim_time', True)

        # Declaring the different publishers
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
        self.tf_br = tf.TransformBroadcaster()
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.ws_pub = wb_pub.WholeBodyStatePublisher('/robot_states', self.fbs)
        
        # Initializing the joint state message
        self.joint_msg = JointState()
        for j in range(self.fbs.getJointDoF()):
            self.joint_msg.name.append(self.fbs.getJointNames()[j])
            self.joint_msg.position.append(0.)
            self.joint_msg.velocity.append(0.)
            self.joint_msg.effort.append(0.)
    
    def update(self, states):
        t = states[0].getTime()
        rtime = rospy.Time(t)
        self.clock_pub.publish(rtime)
        if self.last_pub_time + 1. / self.pub_rate < t:
            # Publish all the ROS messages
            base_pos = states[0].getBasePosition()
            base_rpy = states[0].getBaseRPY()
            self.tf_br.sendTransform((base_pos[dwl.X], base_pos[dwl.Y], base_pos[dwl.Z]),
                tf.transformations.quaternion_from_euler(base_rpy[dwl.X], base_rpy[dwl.Y], base_rpy[dwl.Z]),
                rtime, self.base_link, "world")

            self.joint_msg.header.stamp = rtime
            for j in range(self.fbs.getJointDoF()):
                self.joint_msg.position[j] = states[0].getJointPosition(j)
                self.joint_msg.velocity[j] = states[0].getJointVelocity(j)
                self.joint_msg.effort[j] = states[0].getJointEffort(j)    
            self.joint_pub.publish(self.joint_msg)
            self.ws_pub.publish(states[0])

            # Update the last publication time
            self.last_pub_time = t