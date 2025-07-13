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
from rcl_interfaces.msg import SetParametersResult

from abstract_decision_components.accept.accept_action_server import AcceptActionServer
from abstract_decision_components.accept.accept import satisficing


class AcceptSatisficingActionServer(AcceptActionServer):
    """Accepts a choice if the scores of all chosen alternatives are
    greater than or equal to the threshold values of each requested axis.

    :param axes: A list of strings of axes to check if satisficing. Must be the
        same length as ``thresholds``.
    :param thresholds: A list of doubles of thresholds for each axis. Must be
        the same length as ``axes``.
    """
    def __init__(self):
        super().__init__('satisficing')
        self.add_on_set_parameters_callback(self._check_params)
        self.declare_parameter('axes', Parameter.Type.STRING_ARRAY) 
        self.declare_parameter('thresholds', Parameter.Type.DOUBLE_ARRAY)

    def accept(self, choice):
        axes = self.get_parameter('axes').value
        thresholds = self.get_parameter('thresholds').value

        # TODO: explicilty handle missing axes
        chosen_indices = [choice.evaluation.alternatives.index(c) for c in choice.chosen] 
        axis_indices = [choice.evaluation.axes.index(a) for a in axes] 

        scores = np.array(choice.evaluation.scores).reshape(
                (len(choice.evaluation.alternatives), len(choice.evaluation.axes)))
        return satisficing(scores[np.ix_(chosen_indices, axis_indices)], np.array(thresholds))

    def _check_params(self, parameters):
        # TODO: currently these cannot be changed simultaneously from the command line which
        #   also means that the lengths cannot be modified, once set.

        axes = None
        thresholds = None
        for p in parameters:
            if p.name == 'axes':
                axes = p.value
            elif p.name == 'thresholds':
                thresholds = p.value

        if axes is None and thresholds is None:
            return SetParametersResult(successful=True)

        if axes is None:
            if self.get_parameter_or('axes', False):
                axes = self.get_parameter('axes').value
            else:
                return SetParametersResult(successful=True)
        if thresholds is None:
            if self.get_parameter_or('thresholds', False):
                thresholds = self.get_parameter('thresholds').value
            else:
                return SetParametersResult(successful=True)

        if len(axes) != len(thresholds):
            return SetParametersResult(successful=False,
                                       reason=f'Unequal lengths of axes ({len(axes)}) and thresholds ({len(thresholds)})')

        self.policy_str = f'satisficing, "{list(zip(axes, thresholds))}"'
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    node = AcceptSatisficingActionServer()

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
