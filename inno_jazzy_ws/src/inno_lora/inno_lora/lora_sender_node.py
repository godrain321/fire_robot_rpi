import rclpy
from rclpy.node import Node


class LoraSender(Node):
    def __init__(self) -> None:
        super().__init__('lora_sender')
        self.get_logger().info('[inno_lora] LoRa sender started')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoraSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
