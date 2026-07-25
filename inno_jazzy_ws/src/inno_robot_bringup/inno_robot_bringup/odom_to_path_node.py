#!/usr/bin/env python3
import math
from collections import deque
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        for n,v in [('odom_topic','/odom_rf2o'),('path_topic','/rf2o_path'),('frame_id','odom'),('max_points',5000)]: self.declare_parameter(n,v)
        self.frame=str(self.get_parameter('frame_id').value); self.poses=deque(maxlen=int(self.get_parameter('max_points').value))
        qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=5,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        self.pub=self.create_publisher(Path,str(self.get_parameter('path_topic').value),qos)
        self.sub=self.create_subscription(Odometry,str(self.get_parameter('odom_topic').value),self.cb,qos)
    def cb(self,msg):
        p=PoseStamped(); p.header.stamp=msg.header.stamp; p.header.frame_id=self.frame; p.pose=msg.pose.pose
        q=p.pose.orientation; values=(q.x,q.y,q.z,q.w); norm=math.sqrt(sum(v*v for v in values))
        if not all(math.isfinite(v) for v in values) or norm < 1e-9: q.x=q.y=q.z=0.0; q.w=1.0
        elif abs(norm-1.0)>1e-6: q.x/=norm; q.y/=norm; q.z/=norm; q.w/=norm
        self.poses.append(p); out=Path(); out.header.stamp=msg.header.stamp; out.header.frame_id=self.frame; out.poses=list(self.poses); self.pub.publish(out)
def main(args=None):
    rclpy.init(args=args); node=OdomToPath()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
