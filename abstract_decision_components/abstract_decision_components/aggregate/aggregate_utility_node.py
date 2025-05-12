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

from rclpy.node import Node

from decision_msgs.msg import Evaluation, AssessmentMatrix

from abstract_decision_components.util import validate_matrix


class AggregateUtilityNode(Node):
    """An abstract class to compute a single utility score for each alternative.
    """

    def __init__(self, policy):
        self.policy_ = f'aggregate_utility_{policy}'
        self.policy_str = self.policy_ # mutable by children

        super().__init__(f'{self.policy_}_node')
        self.get_logger().info(f'Starting AGGREGATE node with policy: {self.policy_}')

        self.sub_assesments_ = self.create_subscription(
                AssessmentMatrix,
                'assessments',
                self.assessments_cb,
                10)
        
        self.pub_ = self.create_publisher(
                Evaluation,
                'evaluation',
                10)

    def assessments_cb(self, msg):
        self.get_logger().info(f'Aggregating assessments of {msg.cues} on {msg.alternatives} with policy: {self.policy_str}')

        try:
            validate_matrix(len(msg.alternatives), len(msg.cues), len(msg.scores))
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        try:
            utilities = self.aggregate(msg) # implemented by children
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        self.pub_.publish(Evaluation(alternatives=msg.alternatives,
                                     axes=[f'"{self.policy_str}"'],
                                     scores=utilities))

    # To be overridden by children
    def aggregate(self, msg):
        raise NotImplementedError
        
