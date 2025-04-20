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

from decision_msgs.msg import Evaluation, AssessmentArray, CueWeights, Judgment, Feature
import aggregate


class AggregateUtilityNode(Node):
    """Compute a single utility score for each alternative.
    """
    def __init__(self):
        super().__init__('aggregate_utility_node')
        self.get_logger().info('Starting AGGREGATE node with policy: aggregate_utility')

        self.declare_parameter('boolean_operator', 'and') # An operator for boolean comparisons
        self.declare_parameter('policy', 'unweighted_sum')

        self.sub_weights_ = self.create_subscription(
                CueWeights,
                'cue_weights',
                self.update_weights_cb,
                10)
        self.weights_ = {}

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
        raise NotImplementedError("Not yet been tested")
        policy = self.get_parameter('policy').string_value

        policy_str = policy

        match policy:
            case 'weighted_sum':
                policy_str += f' weights={self.weights_}'
                utilities = self.weighted_sum(msg, self.weights_)
            case 'tallying' | 'unweighted_sum': # these are equivalent if binary cues
                weights = {a.cue.id : 1 for a in msg.assessments}
                policy_str += f' weights={weights}'
                utilities = self.weighted_sum(msg, weights)
            case 'boolean':
                operator = self.get_parameter('boolean_operator').string_value
                policy_str += f' operator={operator}'
                utilities = self.boolean_combination(msg, operator)
            case 'dawes':
                utilities = self.dawes_rule(msg)
            case _:
                self.get_logger().warn(f"Policy '{policy}' not recognized. Defaulting to 'unweighted_sum'")
                weights = {a.cue.id : 1 for a in msg.assessments}
                utilities = self.weighted_sum(msg, weights)

        judgments_map = self._assemble_judgment_map(utilities)

        alternatives = judgments_map.keys()
        axes = [a.cue.id for a in msg.assessments]
        self.get_logger().info(f'Aggregating assessments of {axes} on {alternatives} with policy: aggregate_utility "{policy_str}"')
        self.pub_.publish(Evaluation(judgments=judgments_map.values()))

    def boolean_combination(self, msg, operator):
        try:
            truthiness = aggregate.boolean_combination(msg.assessments, operator)
        except ValueError as e:
            self.get_logger().warn(e + " Defaulting to 'and'")
            truthiness = aggregate.boolean_combination(msg.assessments, 'and')
        return truthiness

    def update_weights_cb(self, msg):
        self.weights_ = {}
        for cue, weight in zip(msg.cues, msg.weights):
            self.weights_[cue.id] = weight

    def weighted_sum(self, msg, weights):
        if len(weights) != len(msg.assessments):
            self.get_logger().error("Incorrect weights for assessment. Defaulting to 'tallying'")
            weights = {a.cue.id : 1 for a in msg.assessments}

        return aggregate.weighted_sum(msg.assessments, weights)

    def dawes_rule(self, msg):
        return aggregate.dawes_rule(msg.assessments)

    def _assemble_judgment_map(self, msg, utilities):
        # Assemble judgments
        alternatives = [p.alternative for p in msg.assessments[0].preferences]
        judgments_map = {}
        for a, u in zip(alternatives, utilities):
            judgments_map.update({a.id : 
                                  Judgment(alternative=a,
                                           features=[Feature(axis='utility',
                                                             score=u)])
                                  })
        return judgments_map
        

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
