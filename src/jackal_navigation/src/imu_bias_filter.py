import rospy
from sensor_msgs.msg import Imu
import numpy as np
from tf.transformations import quaternion_multiply, quaternion_inverse


class ImuBiasFilter:
    def __init__(self):
        """Initialize the IMU Bias Filter Node."""
        rospy.init_node('imu_bias_filter', anonymous=True)
        rospy.loginfo("IMU Bias Filter node started")
        
        # Subscriber to receive IMU bias messages
        self.subscriber = rospy.Subscriber('/imu/data/bias', Imu, self.calculate_mean_bias)
        
        # Subscriber to receive IMU messages
        self.subscriber = rospy.Subscriber('/imu/data', Imu, self.filter_imu)
        
        # Publisher to send filtered IMU bias
        self.publisher = rospy.Publisher('/imu/data/bias_filtered', Imu, queue_size=100)
        
        # Lists to store IMU bias samples
        self.bias_orientation_x = []
        self.bias_orientation_y = []
        self.bias_orientation_z = []
        self.bias_orientation_w = []
        
        self.bias_angular_velocity_x = []
        self.bias_angular_velocity_y = []
        self.bias_angular_velocity_z = []
        
        self.bias_linear_acceleration_x = []
        self.bias_linear_acceleration_y = []
        self.bias_linear_acceleration_z = []
        
        # Mean values (initialized as None)
        self.mean_orientation_x_bias = None
        self.mean_orientation_y_bias = None
        self.mean_orientation_z_bias = None
        self.mean_orientation_w_bias = None
        
        self.mean_angular_velocity_x_bias = None
        self.mean_angular_velocity_y_bias = None
        self.mean_angular_velocity_z_bias = None
        
        self.mean_linear_acceleration_x_bias = None
        self.mean_linear_acceleration_y_bias = None
        self.mean_linear_acceleration_z_bias = None
        
        # Filtered IMU message
        self.processed_imu = None

        # Loop rate
        self.rate = rospy.Rate(50)  # 10 Hz
    
    
    def calculate_mean_bias(self, msg):
        """Callback function to filter the IMU bias."""
        # rospy.loginfo("Received IMU bias message")

        # Store bias values
        self.bias_orientation_x.append(msg.orientation.x)
        self.bias_orientation_y.append(msg.orientation.y)
        self.bias_orientation_z.append(msg.orientation.z)
        self.bias_orientation_w.append(msg.orientation.w)
        
        self.bias_angular_velocity_x.append(msg.angular_velocity.x)
        self.bias_angular_velocity_y.append(msg.angular_velocity.y)
        self.bias_angular_velocity_z.append(msg.angular_velocity.z) 
        
        self.bias_linear_acceleration_x.append(msg.linear_acceleration.x)
        self.bias_linear_acceleration_y.append(msg.linear_acceleration.y)
        self.bias_linear_acceleration_z.append(msg.linear_acceleration.z)
        
        # rospy.loginfo("the length of the bias_orientation_x is %d", len(self.bias_orientation_x))
        # rospy.loginfo("the length of the bias_angular_velocity_x is %d", len(self.bias_angular_velocity_x))
        # rospy.loginfo("the length of the bias_linear_acceleration_x is %d", len(self.bias_linear_acceleration_x))
        
        # Compute the mean bias after collecting 100 samples
        # if len(self.bias_orientation_x) >= 100:
        
        rospy.loginfo("Computing Mean Bias")
        self.mean_orientation_x_bias = np.mean(self.bias_orientation_x)
        self.mean_orientation_y_bias = np.mean(self.bias_orientation_y)
        self.mean_orientation_z_bias = np.mean(self.bias_orientation_z)
        self.mean_orientation_w_bias = np.mean(self.bias_orientation_w)
        
        
        self.mean_angular_velocity_x_bias = np.mean(self.bias_angular_velocity_x)
        self.mean_angular_velocity_y_bias = np.mean(self.bias_angular_velocity_y)
        self.mean_angular_velocity_z_bias = np.mean(self.bias_angular_velocity_z)
        
        self.mean_linear_acceleration_x_bias = np.mean(self.bias_linear_acceleration_x)
        self.mean_linear_acceleration_y_bias = np.mean(self.bias_linear_acceleration_y)
        self.mean_linear_acceleration_z_bias = np.mean(self.bias_linear_acceleration_z)
        
        if len(self.bias_orientation_x) >= 100:
            rospy.loginfo("bias stack cleared")
            self.bias_orientation_x = []
            self.bias_orientation_y = []
            self.bias_orientation_z = []
            self.bias_orientation_w = []
            
            self.bias_angular_velocity_x = []
            self.bias_angular_velocity_y = []
            self.bias_angular_velocity_z = []
            
            self.bias_linear_acceleration_x = []
            self.bias_linear_acceleration_y = []
            self.bias_linear_acceleration_z = []
    
    def filter_imu(self, msg):
        """Callback function to filter the IMU."""
        # rospy.loginfo("Received IMU message")
                # Apply bias correction if means are computed
                
        if self.mean_orientation_x_bias is not None:
            # msg.orientation.x -= self.mean_orientation_x_bias
            # msg.orientation.y -= self.mean_orientation_y_bias
            # msg.orientation.z -= self.mean_orientation_z_bias
            # msg.orientation.w -= self.mean_orientation_w_bias
            
            q_array = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            bias_array = [self.mean_orientation_x_bias, self.mean_orientation_y_bias, self.mean_orientation_z_bias, self.mean_orientation_w_bias]
            q_bias = quaternion_inverse(bias_array)
            q_corrected = quaternion_multiply(q_array, q_bias)
            msg.orientation.x = q_corrected[0]
            msg.orientation.y = q_corrected[1]
            msg.orientation.z = q_corrected[2]
            msg.orientation.w = q_corrected[3]
            
            
            msg.angular_velocity.x -= self.mean_angular_velocity_x_bias
            msg.angular_velocity.y -= self.mean_angular_velocity_y_bias
            msg.angular_velocity.z -= self.mean_angular_velocity_z_bias
            
            msg.linear_acceleration.x -= self.mean_linear_acceleration_x_bias
            msg.linear_acceleration.y -= self.mean_linear_acceleration_y_bias
            msg.linear_acceleration.z -= self.mean_linear_acceleration_z_bias
            
            rospy.loginfo("Filtered IMU:")
            rospy.loginfo("Orientation: x=%.4f, y=%.4f, z=%.4f, w=%.4f", 
                            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            rospy.loginfo("Angular Velocity: x=%.4f, y=%.4f, z=%.4f",
                            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
            rospy.loginfo("Linear Acceleration: x=%.4f, y=%.4f, z=%.4f",
                            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
            
            self.processed_imu = msg  # Update the filtered IMU message
    
    def run(self):
        """Publish the filtered IMU bias."""
        while not rospy.is_shutdown():
            if self.processed_imu is not None:
                rospy.loginfo("Publishing the filtered IMU bias")
                self.publisher.publish(self.processed_imu)
            self.rate.sleep()

if __name__ == '__main__':
    node = ImuBiasFilter()
    node.run()
