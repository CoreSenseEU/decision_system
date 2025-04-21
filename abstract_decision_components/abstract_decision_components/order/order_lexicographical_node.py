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
from rclpy.parameter import Parameter

from decision_msgs.msg import Evaluation, OrderedEvaluation, WeakOrdering
from abstract_decision_components.order import order


class OrderLexicographicalNode(Node):
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
        raise NotImplementedError("Not yet been tested")
        axes = self.get_parameter('axis_ordering').value
        policy = 'lexicographical'
        n_axes_recieved = len(msg.judgments[0].features)

        if n_axes_recieved == 1:
            policy = 'maximize'
            axes = [msg.judgments[0].features[0].axis]
        elif len(axes) == 0:
            self.get_logger().warn(f'Parameter `axis_ordering` is unset. Assuming ordering in recieved evaluation: {msg.axes}')
            axes = msg.axes
        else:
            assert(len(axes) == n_axes_recieved and sorted(axes) == sorted(msg.axes))
        
        alternatives, ranks = order.lexicographical(msg.judgments, axes)
        ordering = WeakOrdering(alternatives=alternatives, ranks=ranks)
        ordered_eval = OrderedEvaluation(ordering=ordering, evaluation=msg)

        self.get_logger().info(f'{alternatives} ordered {ranks} with policy: order_{policy} "{axes}"')
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
