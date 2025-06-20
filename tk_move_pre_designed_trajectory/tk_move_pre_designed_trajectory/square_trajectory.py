import rclpy
from .polygon_trajectory import PolygonTrajectoryNode

class SquareTrajectoryNode(PolygonTrajectoryNode):
    def __init__(self):
        super().__init__('square_trajectory_node', num_sides=4)

def main(args=None):
    rclpy.init(args=args)
    node = SquareTrajectoryNode()
    try:
        node.execute_trajectory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
