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
from rclpy.action import ActionServer

from decision_msgs.msg import Evaluation
from decision_msgs.action import Aggregate

from abstract_decision_components.util import validate_matrix, load_cs_description


class AggregateUtilityActionServer(Node):
    """An abstract class to compute a single utility score for each alternative.
    """

    def __init__(self, policy):
        self.policy_ = f'aggregate_utility_{policy}'
        self.policy_str = self.policy_ # mutable by children

        super().__init__(f'{self.policy_}_action_server')
        self.get_logger().info(f'Starting AGGREGATE action server with policy: {self.policy_}')

        self.declare_parameter('coresense_engine', load_cs_description(self.policy_))

        self.action_server_ = ActionServer(
                self,
                Aggregate,
                '~/Aggregate',
                self.aggregate_cb)

        self.pub_ = self.create_publisher(
                Evaluation,
                'evaluation',
                10)

    def aggregate_cb(self, goal_handle):
        assessments = goal_handle.request.assessments
        self.get_logger().info(f'Aggregating assessments of {assessments.cues} on {assessments.alternatives} with policy: {self.policy_str}')

        try:
            validate_matrix(len(assessments.alternatives), 
                            len(assessments.cues), 
                            len(assessments.scores))
            utilities = self.aggregate(assessments) # implemented by children
        except ValueError as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            return Aggregate.Result()

        evaluation = Evaluation(alternatives=assessments.alternatives,
                                axes=[self.policy_str],
                                scores=utilities)
        self.pub_.publish(evaluation)

        goal_handle.succeed()
        self.get_logger().info(f'Succeeded with evaluation {evaluation}')
        return Aggregate.Result(axes=evaluation.axes,
                                scores=evaluation.scores)

    # To be overridden by children
    def aggregate(self, assessments):
        raise NotImplementedError
        
