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

from decision_interfaces.msg import Choice, OrderedEvaluation
import take


class EliminateWorstNode(Node):
    def __init__(self):
        super().__init__('eliminate_worst_node')
        self.get_logger().info('Starting TAKE server with policy: eliminate_worst')

        self.declare_parameter('n', 0)
        self.declare_parameter('random_ties', False)

        self.sub_ = self.create_subscription(
                OrderedEvaluation,
                'ordered_evaluation',
                self.choice_cb,
                10)
        self.pub_ = self.create_publisher(
                Choice,
                'choice',
                10)

    def choice_cb(self, msg):
        n = self.get_parameter('n').integer_value
        ranked_alternatives = take.create_ordered_pairs(msg.ordering.alternatives, msg.ordering.ranks)

        choice = Choice(evaluation=msg.evaluation)
        if n == 0:
            choice.chosen = self.eliminate_worst(ranked_alternatives)
        elif self.get_parameter('random_ties').bool_value:
            choice.chosen = self.eliminate_n_random(ranked_alternatives)
        else:
            choice.chosen = self.eliminate_n(ranked_alternatives)

        self.pub_.publish(choice)

    def emilinate_n_cb(self, ranked_alternatives, n):
        """
        Eliminates the worst N alternatives.
        """
        chosen = take.eliminate_worst(ranked_alternatives, n=n)

        self.get_logger().info(f'{chosen} taken with policy: eliminate_n, n={n}')
        return chosen

    def eliminate_n_random_cb(self, ranked_alternatives, n):
        """
        Eliminates the worst N alternatives, randomly selecting among tied classes.
        """
        chosen = take.eliminate_worst(ranked_alternatives, n=n, random_ties=False)

        self.get_logger().info(f'{chosen} taken with policy: eliminate_n_random, n={n}')
        return chosen

    def eliminate_worst_cb(self, ranked_alternatives):
        """
        Eliminates all the alternatives tied for worst score.
        """
        chosen = take.eliminate_worst(ranked_alternatives)

        self.get_logger().info(f'{chosen} taken with policy: eliminate_worst')
        return chosen


def main(args=None):
    rclpy.init(args=args)

    node = EliminateWorstNode()

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
