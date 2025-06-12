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

from abstract_decision_components.accept.accept import dominating
from abstract_decision_components.accept.accept_action_server import AcceptActionServer


class AcceptDominatingActionServer(AcceptActionServer):
    """
    Accepts a choice if the scores of all chosen alternatives are strictly
    greater than the best unchosen alternative for each requested axis.

    :param axes: A list of strings of axes to check for dominance.
    """
    def __init__(self):
        super().__init__('dominating')
        self.declare_parameter('axes', Parameter.Type.STRING_ARRAY)

    def accept(self, choice):
        axes = self.get_parameter('axes').value
        
        # TODO: explicilty handle missing axes
        chosen_indices = [choice.evaluation.alternatives.index(c) for c in choice.chosen] 
        axis_indices = [choice.evaluation.axes.index(a) for a in axes] 
        scores = np.array(choice.evaluation.scores).reshape(
                (len(choice.evaluation.alternatives), len(choice.evaluation.axes)))
        return dominating(chosen_indices, scores[:, axis_indices])
 

def main(args=None):
    rclpy.init(args=args)

    node = AcceptDominatingActionServer()

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
