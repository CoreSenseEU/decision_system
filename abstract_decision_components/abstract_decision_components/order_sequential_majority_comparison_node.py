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

import numpy as np

from decision_interfaces.msg import Evaluation, OrderedEvaluation, Ordering
import order


class OrderSequentialMajorityComparisonNode(Node):
    def __init__(self):
        super().__init__('order_sequential_majority_comparison_node')
        self.get_logger().info('Starting ORDER node with policy: order_sequential_majority_comparison')

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
        # TODO: fail gracefully when this doesn't work?
        feature_matrix = np.array(msg.feature_matrix).reshape((len(msg.alternatives),len(msg.axies)))

        ranks = order.sequential_majority_comparison(feature_matrix)
        ordering = Ordering(alternatives=msg.alternatives, ranks=ranks)
        ordered_eval = OrderedEvaluation(ordering=ordering, evaluation=msg)

        self.get_logger().info(f'{msg.alternatives} ordered {ranks} with policy: order_sequential_majority_comparison')
        self.pub_.publish(ordered_eval)


def main(args=None):
    rclpy.init(args=args)

    node = OrderSequentialMajorityComparisonNode()

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
