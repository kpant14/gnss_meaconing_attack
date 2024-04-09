#!/usr/bin/env python3
import rclpy
import numpy as np
import navpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from px4_msgs.msg import VehicleOdometry

class SpooferTraj(Node):
    """
    Move a target entity(gazebo model) along a set trajectory defined by traj_f
    traj_f should always take @t and @begin_pose as the first two arguments
    """
    def __init__(self, target_name="spoofer") -> None:
        self.target_name = target_name
        super().__init__('spoofer_gz_stream')
        ## Configure subscritpions
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.declare_parameter('px4_ns', 'px4_1')
        self.declare_parameter('gz_world_name', 'AbuDhabi')

        
        self.ns = self.get_parameter('px4_ns').get_parameter_value().string_value
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            f'{self.ns}/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile)

        self.gz_world_name = self.get_parameter('gz_world_name').get_parameter_value().string_value
        self.attack_flag_pub = self.create_publisher(Bool, "/attack_flag", 10)
        self.client = self.create_client(SetEntityPose, f'/world/{self.gz_world_name}/set_pose')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gazebo set_entity_state service is not available, waiting...')

        self.lla_ref = np.array([24.484043629238872, 54.36068616768677, 0]) # latlonele -> (deg,deg,m)
        self.attack_wpt_lla = np.array(
           [24.48476311664666, 54.3614948536716, 30])
        self.attack_wpt = navpy.lla2ned(self.attack_wpt_lla[0], self.attack_wpt_lla[1],
                    self.attack_wpt_lla[2],self.lla_ref[0], self.lla_ref[1], self.lla_ref[2],
                    latlon_unit='deg', alt_unit='m', model='wgs84')

        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position_enu = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_position_ned = np.array([0.0, 0.0, 0.0])
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.count = 0 
        self.mode = 0 
        self.attack_flag= False
        self.attack_count = 0
    
    def vector2PoseMsg(self,position, attitude):
        pose_msg = Pose()
        pose_msg.orientation.w = attitude[0]
        pose_msg.orientation.x = attitude[1]
        pose_msg.orientation.y = attitude[2]
        pose_msg.orientation.z = attitude[3]
        pose_msg.position.x = position[0]
        pose_msg.position.y = position[1]
        pose_msg.position.z = position[2]
        return pose_msg
    
    def odom_callback(self,msg):
        self.vehicle_local_position_ned = np.array([msg.position[0],msg.position[1],msg.position[2]],dtype=np.float64)
        self.vehicle_local_position_enu = np.array([msg.position[1], msg.position[0], -msg.position[2]],dtype=np.float64)
        self.vehicle_attitude = np.array([msg.q[0], msg.q[1], -msg.q[2], -msg.q[3]], dtype=np.float64)
        
    
    def cmdloop_callback(self):
        dist_xyz    =   np.sqrt(np.power(self.attack_wpt[0]-self.vehicle_local_position_ned[0],2)+ \
                                    np.power(self.attack_wpt[1]-self.vehicle_local_position_ned[1],2)+ \
                                    np.power(self.attack_wpt[2]-self.vehicle_local_position_ned[2],2))
        # Inject meaconing attack when the drone reaches attack waypoint  
        if dist_xyz <= 10 and not self.attack_flag:
            self.attack_flag = True
            self.get_logger().info('GNSS meaconing attack started')

        
        if self.attack_flag:   
            # Incremental attack scheme with two modes, changing the latitude of the spoofer model
            if self.mode ==0:
                self.count+=1
                if (self.count>500):
                    self.mode=1
            else:
                self.count-=1
                if (self.count<0):
                    self.mode=0      
            spoofer_position = [self.vehicle_local_position_enu[0] , 
                                self.vehicle_local_position_enu[1]+ self.count*0.01, 36.0] 
                         
            vehicle_pose_msg = self.vector2PoseMsg(spoofer_position, self.vehicle_attitude)
            self.request = SetEntityPose.Request()
            entity = Entity()
            entity.name = self.target_name
            self.request.entity = entity
            self.request.pose = vehicle_pose_msg
            future = self.client.call_async(self.request)
            if future.done():
                response = future.result()
            
            # Adding delay for not causing sudden jump in residuals. 
            self.attack_count += 1
            if self.attack_count > 100:    
                bool_msg = Bool()
                bool_msg.data = True
                self.attack_flag_pub.publish(bool_msg)
        
def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    spoofer_traj = SpooferTraj()
    executor.add_node(spoofer_traj)
    executor.spin()

    try:
        rclpy.shutdown()
    except Exception():
        pass

if __name__ == '__main__':
    main()