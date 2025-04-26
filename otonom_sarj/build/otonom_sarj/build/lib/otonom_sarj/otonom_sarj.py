import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String

class ChargeCommander(Node):
    def __init__(self):
        super().__init__('charge_commander')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.subscription = self.create_subscription(String, 'charge_command', self.charge_callback, 10)

        self.charge_x = 4.77
        self.charge_y = 3.2

        self.server_ready = False
        self.wait_for_server()

    def wait_for_server(self):
        self.get_logger().info('Nav2 Action Server\'a bağlanılıyor...')
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 Action Server hazır değil, bekleniyor...')
        self.get_logger().info('Nav2 Action Server hazır!')
        self.server_ready = True

    def charge_callback(self, msg):
        if msg.data == "go_to_charge":
            self.get_logger().info('Şarj istasyonu hedefleniyor...')
            if self.server_ready:
                self.send_goal(self.charge_x, self.charge_y)
            else:
                self.get_logger().error('Action Server hazır değil!')

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        # 180 derece dönük hedef gönderiyoruz
        pose.pose.orientation.z = 1.0
        pose.pose.orientation.w = 0.0

        goal_msg.pose = pose

        self.get_logger().info(f'Hedef gönderiliyor: ({x}, {y})')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Hedef Reddedildi :(')
            return

        self.get_logger().info('Hedef Kabul Edildi, Robot Gidiyor...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info('Robot hedefe başarıyla ulaştı! 🚀')
        else:
            self.get_logger().warn(f'Hedef başarısız tamamlandı. Status: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = ChargeCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
