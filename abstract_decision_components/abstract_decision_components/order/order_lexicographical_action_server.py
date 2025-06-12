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
from rclpy.parameter import Parameter

from abstract_decision_components.order.order import lexicographical
from abstract_decision_components.order.order_action_server import OrderActionServer
from abstract_decision_components.util import scores_to_np_array


class OrderLexicographicalActionServer(OrderActionServer):
    """Orders alternatives lexicographically by score, breaking ties in order
    of axes. Highest score is better.

    :param axis_ordering: A list of strings of axes in the desired order. All
        judgments must contain exactly these axes.
    """
    def __init__(self):
        super().__init__('lexicographical')

        self.declare_parameter('axis_ordering', Parameter.Type.STRING_ARRAY)

    def order(self, msg):
        axes = self.get_parameter('axis_ordering').value
        index_array = []
        for axis in axes:
            if axis in msg.axes:
                index_array.append(msg.axes.index(axis))
            else:
                self.get_logger().warn(f'Evaluation axes {msg.axes} does not contain axis: {axis}')

        if len(index_array) == 0:
            self.get_logger().warn(f'No axes in the evaluation {msg.axes} match ordering parameters: {axes}')
            return [0] * len(msg.alternatives)

        if len(index_array) == 1:
            self.policy_str = 'maximize'
        else:
            self.policy_str = 'lexicographical'

        return lexicographical(scores_to_np_array(msg.scores, len(msg.alternatives)), index_array)


def main(args=None):
    rclpy.init(args=args)

    node = OrderLexicographicalActionServer()

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
