import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int8MultiArray

joint_array1 = [30,110,90,90,100];
joint_array2 = [30,110,120,90,40];
joint_array3 = [30,110,90,90,40];
joint_array4 = [110,110,90,90,40];
joint_array5 = [110,110,120,90,100];
joint_array6 = [110,110,90,90,100];
joint_array7 = [110,110,120,90,40];
joint_array8 = [110,110,90,90,40];
joint_array9 = [30,110,90,90,40];
joint_array10 = [30,110,120,90,100];
joint_array11 = [30,110,90,90,100];

joint_arrays = [joint_array1,joint_array2,joint_array3,joint_array4,joint_array5,joint_array6,joint_array7,joint_array8,joint_array9,joint_array10,joint_array11];


class JointPublisher(Node):

    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(Int8MultiArray, 'joint_state_subscriber', 10)
        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int8MultiArray()
        msg.data = joint_arrays[self.i]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Joint values are: {joint_arrays[self.i]}')
        self.i += 1
        if self.i > 10:
            self.i = 0


def main(args=None):
    rclpy.init(args=args)

    joint_publisher = JointPublisher()

    rclpy.spin(joint_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
