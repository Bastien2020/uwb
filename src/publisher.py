#!/usr/bin/env python3
import time, serial, rospy
#from tf import TransformBroadcaster
from std_msgs.msg import Float64, Header
from custom_msgs.msg import uwb_header, Uwb
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
import numpy as np
from tf.transformations import euler_from_quaternion

class UwbXyzPublisher(object):
    def __init__(self):
        rospy.init_node("iidre_uwb_xyz_publisher")
        #self.tfb = TransformBroadcaster()
        self.serial = None
        self.device_name = rospy.get_param("name", "uwb")
        self.device_port = rospy.get_param("port", "/dev/ttyACM0")
        self.device_frame_id = rospy.get_param("frame_id", "map")
        self.publish_anchors = rospy.get_param("publish_anchors", True)
        
        self.pub = rospy.Publisher('/testAnchor', uwb_header, queue_size=10)
        self.pub2 = rospy.Publisher('/testIMU', Imu, queue_size=10)
        self.pub3 = rospy.Publisher('/Distance', Uwb, queue_size=10)
        self.uwbMsgOld = Uwb()
        
        self.uwbMsg = uwb_header()
        self.uwbHeader = Header()
        self.point = Point()
        self.imu = Imu()
        
        self.gyro = False 
        self.acc = False
        self.magneto = False
        
        self.imuDic = {'gyro':[0, None],
                       'acc':[0, None],
                       'magneto':[0, None],
                       }
        
        
        
        
    
    def connect(self):
        if self.serial is not None:
            try:
                self.serial.close()
            except:
                pass
        
        rospy.loginfo("Connecting to {}...".format(self.device_port))
        self.serial = serial.Serial(self.device_port)
        rospy.loginfo("Connected! Now publishing tf frame '{}' in frame '{}'...".format(self.device_name, self. device_frame_id))

    def run(self):
        while not rospy.is_shutdown():
            try:
                line = self.serial.readline().decode("ascii")
                ts = rospy.Time.now()
                #print(line)
                self.parse(line, ts)
            except (ValueError, IndexError):
                # Ignore the frame in case of any parsing error
                rospy.loginfo("Error when parsing a frame from serial")
            except serial.serialutil.SerialException:
                rospy.logwarn("Device disconnection, retrying...")
                rospy.sleep(2)
                self.connect()
                

    def parse(self, line, ts):
        fb = line.split(":")
        fb_cmd = fb[0]
        fb_data = fb[1].replace("\r\n","").split(",")
        #print(fb_data)
        time = fb_data[0]
        
        if fb_cmd == "+DIST":
            # This is usable even if the device has not been preconfigured with the uwbSupervisor
            # Just triangulate the distance (not done here)
            self.publish_dist(fb_data, ts)
        
        if fb_cmd== "+MQUAT":
            #print("quat")
            self.magneto = True
            self.imuDic['magneto'][0] = ts
            self.imuDic['magneto'][1] = fb_data
            
        if fb_cmd== "+MGYRO":
            #print("gyro")
            self.gyro = True
            self.imuDic['gyro'][0] = ts
            self.imuDic['gyro'][1] = fb_data
            
        if fb_cmd== "+MACC":
            #print("acc")
            self.acc = True
            self.imuDic['acc'][0] = ts
            self.imuDic['acc'][1] = fb_data
        
        if self.magneto and self.gyro and self.acc:
            self.publish_imu()  #no need to pass data as argument, it is contained in the class dictionnary   

            
    def publish_dist(self, fb_data, ts): 
        anchor_id = fb_data[1]
        anchor_dist = fb_data[2]
        anchor_xyz = fb_data[3:6]
        ax_m, ay_m, az_m = map(lambda x: float(x)/100, anchor_xyz)
        
        self.uwbHeader.stamp = ts
        self.uwbMsg.header = self.uwbHeader
        
        self.point.x = ax_m
        self.point.y = ay_m
        self.point.z = az_m
        
        self.uwbMsg.anchorPos = self.point
        
        self.uwbMsg.range= float(anchor_dist)/100
        self.uwbMsg.anchorId= anchor_id
        
        self.pub.publish(self.uwbMsg)
        
        #backward compatibility
        self.uwbMsgOld.anchorId = anchor_id
        self.uwbMsgOld.range=int(anchor_dist)
        self.pub3.publish(self.uwbMsgOld)
         
    
    def publish_imu(self):
        self.magneto = False
        self.gyro = False 
        self.acc = False
        
        ts1 = self.imuDic['gyro'][0]
        ts2 = self.imuDic['acc'][0]
        ts3 = self.imuDic['magneto'][0]
        ts = min(ts1,ts2,ts3)
        tsm = max(ts1,ts2,ts3)
        
        #print("publishing imu", (tsm-ts), ts1, ts2, ts3)
        
        self.imu.header.stamp = ts
        self.imu.header.frame_id = "tag_frame"  #need to publish the transform from base_link to tag_frame
        qw =  float(self.imuDic['magneto'][1][1])
        qx =  float(self.imuDic['magneto'][1][2])
        qy =  float(self.imuDic['magneto'][1][3])
        qz =  float(self.imuDic['magneto'][1][4])
        nrm = np.sqrt(qw**2+qx**2+qy**2+qz**2)
        
        
        #print("euler: ", euler_from_quaternion([qx/nrm, qy/nrm, qz/nrm, qw/nrm]))
        
        #print("norme quat: ", nrm)
        self.imu.orientation.w = qw/nrm 
        self.imu.orientation.x = qx/nrm
        self.imu.orientation.y = qy/nrm
        self.imu.orientation.z = qz/nrm 
        
        self.imu.angular_velocity.x = float(self.imuDic['gyro'][1][1])*np.pi/(16*180)
        self.imu.angular_velocity.y = float(self.imuDic['gyro'][1][2])*np.pi/(16*180)
        self.imu.angular_velocity.z = float(self.imuDic['gyro'][1][3])*np.pi/(16*180)
       
        
        self.imu.linear_acceleration.x = float(self.imuDic['acc'][1][1])/100
        self.imu.linear_acceleration.y = float(self.imuDic['acc'][1][2])/100
        self.imu.linear_acceleration.z = float(self.imuDic['acc'][1][3])/100
        
        self.pub2.publish(self.imu)
        
    ### quat is the reference quaternion of the robot in the base_link. Theta should be aligned with the original yaw of the robot, other angles shold be 0. basically quat should be (cos(\theta/2, 0,0,sin(\theta/2) 
    def calibrateTagFrame(self, quat):
        pass

            ## if self.publish_anchors:
            ##     self.tfb.sendTransform(
            ##         (ax_m, ay_m, az_m), (0, 0, 0, 1),   # device position, quaternion
            ##         rospy.Time.now(),
            ##         anchor_id,
            ##         self.device_frame_id)   
        
        ## elif fb_cmd == "+MPOS":
        ##     # This is usable if device has been preconfigured with the uwbSupervisor
        ##     x, y, z = fb_data[1:4]
        ##     # Convert from centimeters (in the JSON infra file) to meters
        ##     x_m, y_m, z_m = map(lambda x: float(x)/100, [x, y, z])

        ##     self.tfb.sendTransform(
        ##         (x_m, y_m, z_m), (0, 0, 0, 1),   # device position, quaternion
        ##         rospy.Time.now(),
        ##         self.device_name,
        ##         self.device_frame_id)

if __name__ == "__main__":
    node = UwbXyzPublisher()
    node.connect()
    node.run()
