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


class AggregatePreferencesNode(Node):
    """Does not perform aggregation and simply passes preferences along as judgments.
    """
    def __init__(self):
        super().__init__('aggregate_preferences_node')
        self.get_logger().info('Starting AGGREGATE node with policy: aggregate_preferences')

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
        # Assume all assessments use the same alternatives
        judgments_map = {p.alternative.id : Judgment(alternative=p.alternative) for p in msg.assessments[0].preferences}
        axes = []

        for assessment in msg.assessments:
            axis = assessment.cue.id
            axes.append(axis)
            for preference in assessment.preferences:
                judgments_map[preference.alternative.id].features.append(Feature(axis=axis, score=preference.score))

        alternatives = judgments_map.keys()
        self.get_logger().info(f'Aggregating assessments of {axes} on {alternatives} with policy: aggregate_preferences')
        self.pub_.publish(Evaluation(judgments=judgments_map.values()))


def main(args=None):
    rclpy.init(args=args)

    node = AggregatePreferencesNode()

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
