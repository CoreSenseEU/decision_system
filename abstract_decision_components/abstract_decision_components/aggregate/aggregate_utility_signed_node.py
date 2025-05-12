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

from abstract_decision_components.aggregate.aggregate import dawes_rule, tallying
from abstract_decision_components.aggregate.aggregate_utility_node import AggregateUtilityNode


class AggregateUtilitySignedNode(AggregateUtilityNode):
    """Compute a single utility score for each alternative based on the sign
    of each assessment (positive or negative).
    Scores of 0 are assumed to be negative.

    :param policy: A policy to use for computing utility. Valid options are:
        - `'tallying'`: add the number of cue values greater than 0.
        - `'dawes'`: Use dawes rule: calculate the difference between the number of
            positive and negative cue values (0 is assumed to be negative).
        Defaults to 'tallying'
    """
    def __init__(self):
        super().__init__('signed')
        self.declare_parameter('policy', 'tallying')

    def aggregate(self, msg):
        policy = self.get_parameter('policy').value
        assessments = np.array(msg.scores).reshape((len(msg.alternatives), -1))
        match policy:
            case 'dawes':
                utilities = dawes_rule(assessments)
            case 'tallying':
                utilities = tallying(assessments)
            case _:
                # TODO: move this to the parameter update function...
                raise ValueError(f"Policy '{policy}' invalid. Valid options are" \
                                + "[tallying, dawes]")
        return utilities
        

def main(args=None):
    rclpy.init(args=args)

    node = AggregateUtilitySignedNode()

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
