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
from abstract_decision_components.accept import accept


class AcceptSizeNode(Node):
    """Accepts a choice based on the number of chosen alternatives.

    :param n: An integer size to compare to. Defaults to 1
    :param relation: A relational operator with the rhs of ``choice`` on the
        right hand size and ``rhs`` on the left. Valid relation operators are:
            `<`, `>`, `<=`, `>=`, `=`, and `!=`.
        Defaults to '='
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
        decision = Decision(choice=msg.chosen)
        n = self.get_parameter('n').value
        relation = self.get_parameter('relation').value
        try:
            decision.success, decision.reason = accept.compare_size(msg.chosen, n, relation=relation)
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        if decision.success:
            verb = 'Accepting'
        else:
            verb = 'Rejecting'
        self.get_logger().info(f'{verb} choice {msg.chosen} with policy: accept_n_relational, "|choice| {relation} {n}"')
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
