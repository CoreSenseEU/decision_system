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
from rclpy.action import ActionServer

from decision_msgs.msg import Evaluation
from decision_msgs.action import Aggregate

from abstract_decision_components.util import validate_matrix, load_cs_description


class AggregatePreferencesActionServer(Node):
    """
    Simply passes preferences along as judgments.
    """
    def __init__(self):
        self.policy_ = 'aggregate_preferences'
        super().__init__(f'{self.policy_}_action_server')
        self.get_logger().info('Starting AGGREGATE action server with policy: aggregate_preferences')

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
        self.get_logger().info(f'Aggregating assessments of {assessments.cues} on {assessments.alternatives} with policy: aggregate_preferences')

        try:
            validate_matrix(len(assessments.alternatives), 
                            len(assessments.cues), 
                            len(assessments.scores))
        except ValueError as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            return Aggregate.Result()

        evaluation = Evaluation(alternatives=assessments.alternatives,
                                axes=[c.id for c in assessments.cues],
                                scores=assessments.scores)
        self.pub_.publish(evaluation)

        goal_handle.succeed()
        self.get_logger().info(f'Succeeded with evaluation {evaluation}')
        return Aggregate.Result(axes=evaluation.axes,
                                scores=evaluation.scores)


def main(args=None):
    rclpy.init(args=args)

    node = AggregatePreferencesActionServer()

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
