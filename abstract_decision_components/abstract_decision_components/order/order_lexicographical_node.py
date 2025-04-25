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

from decision_msgs.msg import Evaluation, OrderedEvaluation, WeakOrdering
from abstract_decision_components.order import order


class OrderLexicographicalNode(Node):
    """Orders alternatives lexicographically by score, breaking ties in order
    of axes. Highest score is better.

    :param axis_ordering: A list of strings of axes in the desired order. All
        judgments must contain exactly these axes.
    """
    def __init__(self):
        super().__init__('order_lexicographical_node')
        self.get_logger().info('Starting ORDER node with policy: order_lexicographical')

        self.declare_parameter('axis_ordering', Parameter.Type.STRING_ARRAY)

        self.sub_ = self.create_subscription(
                Evaluation,
                'evaluation',
                self.evaluation_cb,
                10)
        self.pub_ = self.create_publisher(
                OrderedEvaluation,
                'ordered_evaluation',
                10)

    def evaluation_cb(self, msg):
        if len(msg.judgments) < 1:
            self.get_logger.error('Recieved empty list of judgments')
            return

        axes = self.get_parameter('axis_ordering').value
        try:
            alternatives, ranks = order.lexicographical(msg.judgments, axes)
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        if len(axes) == 1:
            policy = 'maximize'
        else:
            policy = 'lexicographical'
        self.get_logger().info(f'{alternatives} ordered {ranks} with policy: order_{policy} "{axes}"')

        ordering = WeakOrdering(alternatives=alternatives, ranks=ranks)
        ordered_eval = OrderedEvaluation(ordering=ordering, evaluation=msg.judgments)
        self.pub_.publish(ordered_eval)


def main(args=None):
    rclpy.init(args=args)

    node = OrderLexicographicalNode()

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
