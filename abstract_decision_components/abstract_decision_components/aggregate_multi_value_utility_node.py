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

from decision_msgs.msg import Evaluation, AssessmentArray


class AggregateMultiValueUtilityNode(Node):
    """Compute multi-valued utility scores for each alternative.
    """
    def __init__(self):
        super().__init__('aggregate_multi_value_utility_node')
        self.get_logger().info('Starting AGGREGATE node with policy: aggregate_multi_value_utility')

        self.declare_parameter('features', []) # A list of features of final judgments
        # TODO: how to specify which assessments to use for which features??
        # - [I] Maybe use separate aggregators and some other node to combine them <-- probably the easiest option

        self.declare_parameter('boolean_operator', []) # An operator for boolean comparisons per assessment
        self.declare_parameter('policy', 'weighted_sum')

        self.sub_choice_ = self.create_subscription(
                AssessmentArray,
                'assessments',
                self.choice_cb,
                10)
        
        self.pub_ = self.create_publisher(
                Evaluation,
                'evaluation',
                10)

    def choice_cb(self, msg):
        raise NotImplementedError("Not yet been tested")
        policy = self.get_parameter('policy').string_value
        match policy:
            case 'weighted_sum':
                ...
            case 'tallying':
                ...
            case 'boolean':
                ...
            case 'dawes':
                ...
            case _:
                self.get_logger().warn("Policy not recognized. Defaulting to 'weighted_sum'")

        # # Assume all assessments use the same alternatives
        # judgments_map = {p.alternative.id : Judgment(alternative=p.alternative) for p in msg.assessments[0].preferences}
        # axes = []
        #
        # for assessment in msg.assessments:
        #     axis = assessment.cue.id
        #     for preference in assessment.preferences:
        #         judgments_map[preference.alternative.id].features.append(Feature(axis=axis, score=preference.score))

        self.get_logger().info(f'Aggregating assessments of {axes} on {alternatives} with policy: aggregate_multi_value_utility')
        self.pub_.publish(evaluation)


def main(args=None):
    rclpy.init(args=args)

    node = AggregateMultiValueUtilityNode()

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
