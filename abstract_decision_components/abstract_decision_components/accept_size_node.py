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

from decision_interfaces.msg import Choice, Decision
import accept


class AcceptSizeNode(Node):
    """
    Accepts a choice based on the number of chosen alternatives.
    """
    def __init__(self):
        super().__init__('accept_size_node')
        self.get_logger().info('Starting ACCEPT node with policy: accept_n_relational')

        self.declare_parameter('n', 1)
        self.declare_parameter('relation', '=')

        self.sub_ = self.create_subscription(
                Choice,
                'choice',
                self.choice_cb,
                10)
        self.pub_ = self.create_publisher(
                Decision,
                'decision',
                10)

    def choice_cb(self, msg):
        decision = Decision(choice=msg.choice)
        n = self.get_parameter('n').integer_value
        relation = self.get_parameter('relation').string_value

        try:
            decision.result, decision.success = accept.compare_size(msg.choice, n, relation=relation)
        except ValueError as e:
            self.get_logger().warn(e + " Defaulting to '='")
            decision.result, decision.success = accept.compare_size(msg.choice, n)

        if decision.success:
            verb = 'Accepting'
        else:
            verb = 'Rejecting'
        self.get_logger().info(f'{verb} choice {msg.choice} with policy: accept_n_relational, "|choice| {relation} {n}"')
        self.pub_.publish(decision)


def main(args=None):
    rclpy.init(args=args)

    node = AcceptSizeNode()

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
