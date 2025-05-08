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
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from decision_msgs.msg import Evaluation, AssessmentArray


class AggregateMultiValueUtilityNode(Node):
    """Compute multi-valued utility scores for each alternative.

    :param children: Namespaces of each of the child aggregators that
        this multi-value utility agregator combines. Each of these children
        must publish its `evaluations` topic under its own namespace.
    """

    def __init__(self, children):
        super().__init__('aggregate_multi_value_utility_node')
        self.get_logger().info(f'Starting AGGREGATE node with policy: {self.policy_}')

        self.subs_ = []
        self.evaluations_buffer_ = None

        self.add_on_set_parameters_callback(self._update_children)
        self.declare_parameter('children', Parameter.Type.STRING_ARRAY)

        self.pub_evaluation_ = self.create_publisher(
                Evaluation,
                'evaluation',
                10)

        self.pub_assessments_ = self.create_publisher(
                    AssessmentArray,
                    '~/assessments',
                    10)

    def evaluations_cb(self, msg, child):
        # Collect evaluations from each
        if len(self.evaluations_buffer_[child]) == 0:
            self.recieved_ += 1
        self.evaluations_buffer_[child].append(msg)

        if self.recieved_ == len(self.children_):
            self.publish_evaluations_()
            self.recieved_ = 0

    def publish_evaluations_(self):
        new_judgments = {}

        # Combine into a single evaluation and publish
        judgments = []
        for child in self.children_:
            judgments += self.evaluations_buffer_[child].pop(0).judgments

        for judgment in judgments:
            key = judgment.alternative.id
            if key not in new_judgments:
                new_judgments.update({key : judgment})
            else:
                new_judgments[key].features += judgment.features

        self.pub_evaluation_.publish(Evaluation(judgments=new_judgments.values()))

    def _update_children(self, parameters):
        children = None
        for p in parameters:
            if p.name == 'children':
                children = p.value
                break
        if children is None:
            return SetParametersResult(successful=True)

        # destroy old subscriptions
        for sub in self.subs_:
            while not self.destroy_subscription(sub):
                self.executor.spin_once()

        # Subscribe to each child
        self.get_logger().info(f'Creating subscriptions to children: {children}')
        self.children_ = children
        self.subs_ = []
        for child in self.children_:
            def cb(self, msg):
                return self.evaluations_cb(msg, child)

            sub = self.create_subscription(
                    Evaluation,
                    f'{child}/evaluations/',
                    cb,
                    10) 
            self.subs_.append(sub)

        # Reset accumulator
        self.evaluations_buffer_ = {c : [] for c in children}
        self.recieved_ = 0

        return SetParametersResult(successful=True)


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
