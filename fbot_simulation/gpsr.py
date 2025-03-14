from gazebo_msgs.srv import SpawnEntity
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os

class GPSRSpawnerNode(Node):
    def __init__(self):
        super().__init__('gpsr_spawner_node')
        self.spawner_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, trying again in 1 second ...')
        self.req = SpawnEntity.Request()

    def spawn_robot(self):
        self.req.name = "shark-v1"
        fbot_simulation_package = get_package_share_directory('fbot_simulation')
        urdf_path = os.path.join(fbot_simulation_package, 'urdf', 'boris.urdf')
        with open(urdf_path, 'r') as f:
            urdf_content = f.read()
        self.req.xml = urdf_content
        return self.spawner_client.call(self.req)

def main(args=None):
    rclpy.init(args=args)
    gpsr_spawner = GPSRSpawnerNode()
    res: SpawnEntity.Response = gpsr_spawner.spawn_robot()
    gpsr_spawner.get_logger().info(res.success)
    gpsr_spawner.get_logger().info(res.status_message)
    #gpsr_spawner.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()
