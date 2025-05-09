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
from rclpy.exceptions import ParameterException

from abstract_decision_components.accept.accept_node import AcceptNode
from abstract_decision_components.accept.accept import satisficing


class AcceptSatisficingNode(AcceptNode):
    """Accepts a choice if the scores of all chosen alternatives are
    greater than or equal to the threshold values of each requested axis.

    :param axes: A list of strings of axes to check if satisficing. Must be the
        same length as ``thresholds``.
    :param thresholds: A list of doubles of thresholds for each axis. Must be
        the same length as ``axes``.
    """
    def __init__(self):
        super().__init__('satisficing')
        self.declare_parameter('axes', Parameter.Type.STRING_ARRAY) 
        self.declare_parameter('thresholds', Parameter.Type.DOUBLE_ARRAY)

    def accept(self, msg):
        axes = self.get_parameter('axes').value
        thresholds = self.get_parameter('thresholds').value
        self._check_params(axes, thresholds)

        chosen_indices = [msg.evaluation.alternatives.index(c) for c in msg.chosen] 
        axis_indices = [msg.evaluation.axes.index(a) for a in axes] 

        scores = np.array(msg.evaluation.scores).reshape(
                (len(msg.evaluation.alternatives), len(msg.evaluation.axes)))
        return satisficing(scores[chosen_indices, axis_indices], np.array(thresholds))

    def _check_params(self, axes, thresholds):
        if len(axes) != len(thresholds):
            raise ParameterException('Parameters have unequal lengths', ['axes', 'thresholds'])
        self.policy_str = f'satisficing, "{list(zip(axes, thresholds))}"'


def main(args=None):
    rclpy.init(args=args)

    node = AcceptSatisficingNode()

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
