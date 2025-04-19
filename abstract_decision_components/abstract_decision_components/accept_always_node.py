# Copyright 2025 KAS-Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys

import rclpy
from rclpy.node import Node

from decision_msgs.msg import Choice, Decision


class AcceptAlwaysNode(Node):
    """
    Always accepts the choice no matter what it is.
    """
    def __init__(self):
        super().__init__('accept_always_node')
        self.get_logger().info('Starting ACCEPT node with policy: accept_always')

        self.sub_choice_ = self.create_subscription(
                Choice,
                'choice',
                self.choice_cb,
                10)
        
        self.pub_ = self.create_publisher(
                Decision,
                'decision',
                10)

    def choice_cb(self, msg):
        self.get_logger().info(f'Accepting choice {msg.chosen} with policy: accept_always')
        self.pub_.publish(Decision(choice=msg.chosen, success=True, reason='Always accepted'))


def main(args=None):
    rclpy.init(args=args)

    node = AcceptAlwaysNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
