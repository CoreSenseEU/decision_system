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

from abstract_decision_components.order.order import pareto_fronts, majority_rule
from abstract_decision_components.order.order_node import OrderNode
from abstract_decision_components.util import scores_to_np_array


class OrderDominatingNode(OrderNode):
    """Orders alternatives based on how many others they dominate. Highest score is better.

    :param strict: If true, assume axis features must be strictly greater 
        to be considered dominating. Defaults to False
    :param policy: A policy to use to compare dominators.
        - `'majority_rule'`: rank each alternative by the number of dominating features it has.
        - `'pareto_fronts'`: rank each alternative by the pareto front it belongs to.
        Defaults to 'majority_rule'

    """
    def __init__(self):
        super().__init__('dominating')
        self.declare_parameter('strict', False)
        self.declare_parameter('policy', 'majority_rule')

    def order(self, msg):
        """
        :raises ValueError: if the ``policy`` parameter is invalid.
        """
        strict = self.get_parameter('strict').value
        policy = self.get_parameter('policy').value

        feature_matrix = scores_to_np_array(msg.scores, len(msg.alternatives))
        match policy:
            case 'pareto_fronts':
                ranks = pareto_fronts(feature_matrix, strict=strict)
            case 'majority_rule':
                ranks = majority_rule(feature_matrix, strict=strict)
            case _:
                # TODO: move to parameter update validation
                raise ValueError(
                        f"Policy '{policy}' invalid. Valid options are "
                         "[majority_rule, pareto_fronts]")

        if strict:
            self.policy_str = f'{policy}, strict'
        else:
            self.policy_str = policy

        return ranks


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
