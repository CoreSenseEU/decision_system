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

import numpy as np

import rclpy
from rclpy.parameter import Parameter

from abstract_decision_components.order.order import lexicographical
from abstract_decision_components.order.order_node import OrderNode


class OrderLexicographicalNode(OrderNode):
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
        try:
            index_array = np.argsort([axes.index(axis) for axis in msg.axes])
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        if len(axes) == 1:
            self.policy_str = 'maximize'
        else:
            self.policy_str = 'lexicographical'

        return lexicographical(msg.scores, index_array)


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
