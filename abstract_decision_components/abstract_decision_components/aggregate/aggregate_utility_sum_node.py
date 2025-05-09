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

from decision_msgs.msg import CueWeights
from abstract_decision_components.aggregate.aggregate import weighted_sum
from abstract_decision_components.aggregate.aggregate_utility_node import AggregateUtilityNode


class AggregateUtilitySumNode(AggregateUtilityNode):
    """Compute a single utility score for each alternative by summing assessments
    for each alternative (optionally weighted by cue).

    :param policy: A policy to use for computing utility. Valid options are:
        - `'unweighted_sum'`: add assessment scores directly.
        - `'weighted_sum'`: add assessment scores based on `cue_weights`.
        Defaults to 'unweighted_sum'
    :param normalize: If True, linearly transform each assessment over its to
        the range `[0,1]`. Defaults to False

    """
    def __init__(self):
        super().__init__('sum')

        self.declare_parameter('policy', 'unweighted')
        self.declare_parameter('normalize', False)

        self.sub_weights_ = self.create_subscription(
                CueWeights,
                'cue_weights',
                self.update_weights_cb,
                10)
        self.weights_ = {}

    def aggregate(self, msg):
        """
        :raises ValueError: if there is a mismatch between the cue weights
            and assessed cues
        """
        policy = self.get_parameter('policy').value

        match policy:
            case 'weighted_sum':
                # TODO: move this to parameter update function
                self.policy_str = f'sum, weights={self.weights_}'

                weights = [self.weights_[cue] for cue in msg.cues]
            case 'unweighted_sum':
                self.policy_str = 'sum, unweighted'
                weights = [1] * len(msg.cues)
            case _:
                # TODO: move this to parameter update function
                raise ValueError(f"Policy '{policy}' invalid. Valid options are" \
                                + "[weighted_sum, unweighted_sum, tallying, boolean, dawes]")

        normalize = self.get_parameter('normalize').value
        assessments = np.array(msg.scores).reshape((len(msg.alternatives), []))
        utilities = weighted_sum(assessments, weights, normalize=normalize)
        return utilities

    def update_weights_cb(self, msg):
        if len(msg.weights) == len(msg.cues):
            self.weights_ = {}
            for cue, weight in zip(msg.cues, msg.weights):
                self.weights_[cue.id] = weight
            self.get_logger().info(f'Recieved new weights: {self.weights_}')
            self.policy_str = f'sum, weights={self.weights_}'
        else:
            self.get_logger().error(
                    f"Mismatch between the number of cues ({len(msg.cues)})"\
                  + f" and weights ({len(msg.weights)})")


def main(args=None):
    rclpy.init(args=args)

    node = AggregateUtilitySumNode()

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
