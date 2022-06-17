"""
 This file is a service client implementation for Gazebo spawn service(SpawnEntity)
 We utilize it for our traffic light sdf files to be spawned into gazebo world
 with specific timing through bash file

 Date  : 9/7/2021
    
"""

import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity
def main():
    argv = sys.argv[1:]
    rclpy.init()
    node = rclpy.create_node("Spawning_Node")
    client = node.create_client(SpawnEntity, "/spawn_entity")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("conencted to spawner")
    sdf_path = argv[0]
    request = SpawnEntity.Request()
    request.name = argv[1]
    request.initial_pose.position.x = float(argv[2])
    request.initial_pose.position.y = float(argv[3])
    request.xml = open(sdf_path, 'r').read()

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()