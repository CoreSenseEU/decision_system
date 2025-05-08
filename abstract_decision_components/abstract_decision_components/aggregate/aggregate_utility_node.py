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

from decision_msgs.msg import Evaluation, AssessmentArray, Judgment, Feature


class AggregateUtilityNode(Node):
    """An abstract class to compute a single utility score for each alternative.
    """

    def __init__(self, policy):
        self.policy_ = f'aggregate_utility_{policy}'
        self.policy_str = self.policy_ # mutable by children

        super().__init__(f'{self.policy_}_node')
        self.get_logger().info(f'Starting AGGREGATE node with policy: {self.policy_}')

        self.sub_assesments_ = self.create_subscription(
                AssessmentArray,
                'assessments',
                self.assessments_cb,
                10)
        
        self.pub_ = self.create_publisher(
                Evaluation,
                'evaluation',
                10)

    def assessments_cb(self, msg):
        if len(msg.assessments) < 1:
            self.get_logger().error('Recieved empty list of assessments')
            return

        try:
            utilities = self.aggregate_assessments(msg.assessments) # implemented by children
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        # Assemble judgments
        alternatives = [p.alternative for p in msg.assessments[0].preferences]
        judgments_map = {}
        for a, u in zip(alternatives, utilities):
            judgments_map.update({a.id : 
                                  Judgment(alternative=a,
                                           features=[Feature(axis=f'"{self.policy_str}"',
                                                             score=u)])
                                  })

        axes = [a.cue.id for a in msg.assessments]
        self.get_logger().info(f'Aggregating assessments of {axes} on {alternatives} with policy: {self.policy_str}')
        self.pub_.publish(Evaluation(judgments=list(judgments_map.values())))

    # To be overridden by children
    def aggregate(self, assessments):
        raise NotImplementedError
        

def main(args=None):
    rclpy.init(args=args)

    node = AggregateUtilityNode()

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
