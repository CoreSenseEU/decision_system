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
import order


class OrderDominatingNode(Node):
    def __init__(self):
        super().__init__('order_dominating_node')
        self.get_logger().info('Starting ORDER node with policy: order_dominating')

        self.declare_parameter('strict', False)
        self.declare_parameter('policy', 'majority_rule')

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
        raise NotImplementedError("Not yet been tested")
        strict = self.get_parameter('strict').bool_value
        policy = self.get_parameter('policy').string_value
        match policy:
            case 'pareto_fronts':
                alternatives, ranks = order.pareto_fronts(msg.judgments)
            case 'majority_rule':
                alternatives, ranks = order.majority_rule(msg.judgments, strict=strict)
            case _:
                self.get_logger().warn("Policy not recognized. Defaulting to 'majority_rule'")
                alternatives, ranks = order.majority_rule(msg.judgments, strict=strict)

        ordering = WeakOrdering(alternatives=alternatives, ranks=ranks)
        ordered_eval = OrderedEvaluation(ordering=ordering, evaluation=msg.judgments)

        self.get_logger().info(f'{msg.alternatives} ordered {ranks} with policy: order_{policy}')
        self.pub_.publish(ordered_eval)


def main(args=None):
    rclpy.init(args=args)

    node = OrderDominatingNode()

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
