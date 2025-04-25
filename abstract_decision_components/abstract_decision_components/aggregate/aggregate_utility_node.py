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
from abstract_decision_components.aggregate import aggregate


class AggregateUtilityNode(Node):
    # TODO: consider splitting this into three different nodes: 
    #   - tallying/dawes
    #   - weighted/unweighted
    #   - boolean
    """Compute a single utility score for each alternative.

    :param policy: A policy to use for computing utility. Valid options are:
        - `'unweighted_sum'`: add assessment scores directly.
        - `'weighted_sum'`: add assessment scores based on `cue_weights`.
        - `'tallying'`: add the number of cue values greater than 0.
        - `'dawes'`: Use dawes rule: calculate the difference between the number of
            positive and negative cue values (0 is assumed to be negative).
        - `'boolean'`: interpret scores as booleans and combine using a boolean
            operator set with the ``boolean_operator`` parameter. Assumes scores
            `> 0` are `True` and `True` values are always preferred.
        Defaults to 'unweighted_sum'
    :param normalize: If ``policy`` is either ``unweighted_sum`` or
        ``weighted_sum`` and this is True, linearly transform each assessment
        over its to the range `[0,1]`. Defaults to False

    :param boolean_operator: If ``policy`` is `boolean`, use this logical
        operator to combine assessments. Valid options are `'or'` or `'and'`,
        defaults to 'and'
    """
    def __init__(self):
        super().__init__('aggregate_utility_node')
        self.get_logger().info('Starting AGGREGATE node with policy: aggregate_utility')

        self.declare_parameter('boolean_operator', 'and')
        self.declare_parameter('policy', 'unweighted_sum')
        self.declare_parameter('normalize', False)

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
        if len(msg.assessments) < 1:
            self.get_logger().error('Recieved empty list of assessments')
            return

        policy = self.get_parameter('policy').value

        policy_str = policy

        try:
            match policy:
                case 'weighted_sum':
                    policy_str += f' weights={self.weights_}'
                    utilities = self.weighted_sum(msg, self.weights_)
                case 'unweighted_sum':
                    weights = {a.cue.id : 1 for a in msg.assessments}
                    utilities = self.weighted_sum(msg, weights)
                case 'boolean':
                    operator = self.get_parameter('boolean_operator').value
                    policy_str += f' operator={operator}'
                    utilities = self.boolean_combination(msg, operator)
                case 'tallying':
                    utilities = self.tallying(msg)
                case 'dawes':
                    utilities = self.dawes_rule(msg)
                case _:
                    raise ValueError(f"Policy '{policy}' invalid. Valid options are" \
                                    + "[weighted_sum, unweighted_sum, tallying, boolean, dawes]")
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        judgments_map = self._assemble_judgment_map(msg, utilities)
        alternatives = judgments_map.keys()
        axes = [a.cue.id for a in msg.assessments]
        self.get_logger().info(f'Aggregating assessments of {axes} on {alternatives} with policy: aggregate_utility "{policy_str}"')
        self.pub_.publish(Evaluation(judgments=list(judgments_map.values())))

    def boolean_combination(self, msg, operator):
        return aggregate.boolean_combination(msg.assessments, operator)

    def update_weights_cb(self, msg):
        if len(msg.weights) != len(msg.cues):
            self.weights_ = {}
            for cue, weight in zip(msg.cues, msg.weights):
                self.weights_[cue.id] = weight
            self.get_logger().info(f'Recieved new weights: {self.weights_}')
        else:
            self.get_logger().error(
                    f"Mismatch between the number of cues ({len(msg.cues)})"\
                  + f" and weights ({len(msg.weights)})")

    def weighted_sum(self, msg, weights):
        """
        :raises ValueError: if there is a mismatch between the cue weights
            and assessed cues
        """
        cues = sorted([a.cue.id for a in msg.assessments])
        weight_keys = sorted(weights.keys())
        if len(weight_keys) != len(cues) and weight_keys != cues:
            raise ValueError(f"Mismatch between the cue weights {weight_keys}" \
                           + f" and assessments {cues}")

        normalize = self.get_parameter('normalize').value
        return aggregate.weighted_sum(msg.assessments, weights, normalize=normalize)

    def dawes_rule(self, msg):
        return aggregate.dawes_rule(msg.assessments)

    def tallying(self, msg):
        return aggregate.tallying(msg.assessments)

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
