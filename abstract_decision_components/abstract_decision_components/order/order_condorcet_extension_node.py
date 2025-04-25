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


class OrderCondorcetExtensionNode(Node):
    """Orders alternatives based on how many others they dominate. Highest score is better.

    :param policy: A policy to use to compare dominators.
        - `'copeland'`: rank each alternative by difference in the number of
          others that are worse than it and better than it in each feature.
        - `'sequential_majority_comparison'`: get single winning alternative by
          making pairwise comparisons of the net preferences of all
          alternatives two at a time.
        - `'majority_of_confirming_dimensions'`: get single winning alternative
          by making pairwise comparisons of the confirming dimensions of
          alternatives two at a time.
        Defaults to 'copeland'

    """
    def __init__(self):
        super().__init__('order_condorcet_extension_node')
        self.get_logger().info('Starting ORDER node with policy: order_condorcet_extension')

        self.declare_parameter('policy', 'copeland')

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
            self.get_logger().error('Recieved empty list of judgments')
            return

        policy = self.get_parameter('policy').value
        match policy:
            case 'copeland':
                alternatives, ranks = order.copeland_method(msg.judgments)
            case 'sequential_majority_comparison':
                alternatives, ranks = order.sequential_majority_comparison(msg.judgments)
            case 'majority_of_confirming_dimensions':
                raise NotImplementedError("Not yet tested")
                alternatives, ranks = order.sequential_majority_comparison(msg.judgments, confirming=True)
            case _:
                self.get_logger().error(
                        f"Policy '{policy}' invalid. Valid options are " \
                         "[copeland, sequential_majority_comparison, majority_of_confirming_dimensions]")
                return

        ordering = WeakOrdering(alternatives=alternatives, ranks=ranks)
        ordered_eval = OrderedEvaluation(ordering=ordering, evaluation=msg.judgments)

        self.get_logger().info(f'{alternatives} ordered {ranks} with policy: order_{policy}')
        self.pub_.publish(ordered_eval)


def main(args=None):
    rclpy.init(args=args)

    node = OrderCondorcetExtensionNode()

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
