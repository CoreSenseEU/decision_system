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

from abstract_decision_components.order.order import copeland_method, sequential_majority_comparison
from abstract_decision_components.order.order_node import OrderNode
from abstract_decision_components.util import scores_to_np_array


class OrderCondorcetExtensionNode(OrderNode):
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
        super().__init__('condorcet_extension')
        self.declare_parameter('policy', 'copeland')

    def order(self, msg):
        """
        :raises ValueError: if the ``policy`` parameter is invalid.
        """
        policy = self.get_parameter('policy').value

        feature_matrix = scores_to_np_array(msg.scores, len(msg.alternatives))
        match policy:
            case 'copeland':
                ranks = copeland_method(feature_matrix)
            case 'sequential_majority_comparison':
                ranks = sequential_majority_comparison(feature_matrix)
            case 'majority_of_confirming_dimensions':
                ranks = sequential_majority_comparison(feature_matrix, confirming=True)
            case _:
                # TODO: move to parameter update validation
                raise ValueError(
                        f"Policy '{policy}' invalid. Valid options are " \
                         "[copeland, sequential_majority_comparison, majority_of_confirming_dimensions]")

        self.policy_str = policy
        return ranks


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
