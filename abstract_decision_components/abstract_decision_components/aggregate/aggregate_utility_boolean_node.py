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

from abstract_decision_components.aggregate.aggregate import boolean_combination
from abstract_decision_components.aggregate.aggregate_utility_node import AggregateUtilityNode
from abstract_decision_components.util import scores_to_np_array


class AggregateUtilityBooleanNode(AggregateUtilityNode):
    """Compute a single utility score for each alternative, combining assessment
    scores with a boolean operator.
    Interpret scores as booleans and combine using a boolean operator set with
    the ``boolean_operator`` parameter. Assumes scores `> 0` are `True` and
    `True` values are always preferred.

    :param boolean_operator: Use this logical operator to combine assessments.
        Valid options are `'or'` or `'and'`, defaults to 'and'
    """
    def __init__(self):
        super().__init__('boolean')
        self.declare_parameter('boolean_operator', 'and')

    def aggregate(self, msg):
        operator = self.get_parameter('boolean_operator').value
        self.policy_str = f'boolean, operator={operator}'
        
        assessments = scores_to_np_array(msg.scores, len(msg.alternatives))
        return boolean_combination(assessments, operator)
        

def main(args=None):
    rclpy.init(args=args)

    node = AggregateUtilityBooleanNode()

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
