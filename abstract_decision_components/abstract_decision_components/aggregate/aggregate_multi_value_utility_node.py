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
import functools

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from decision_msgs.msg import Evaluation, AssessmentMatrix
from abstract_decision_components.util import scores_to_np_array


class AggregateMultiValueUtilityNode(Node):
    """Compute multi-valued utility scores for each alternative.

    :param children: Namespaces of each of the child aggregators that
        this multi-value utility agregator combines. Each of these children
        must publish its `evaluations` topic under its own namespace.
    """
    # TODO: if we switch from topics to services, then this behavior looks very
    #    similar to AssessNode and doesn't require buffering

    def __init__(self):
        super().__init__('aggregate_multi_value_utility_node')
        self.get_logger().info('Starting AGGREGATE node with policy: aggregate_multi_value_utility')

        self.subs_ = []
        self.evaluations_buffer_ = None

        self.add_on_set_parameters_callback(self._update_children)
        self.declare_parameter('children', Parameter.Type.STRING_ARRAY)

        self.pub_evaluation_ = self.create_publisher(
                Evaluation,
                'evaluation',
                10)

        self.pub_assessments_ = self.create_publisher(
                    AssessmentMatrix,
                    '~/assessments',
                    10)

    def evaluations_cb(self, child, msg):
        # Collect evaluations from each
        if len(self.evaluations_buffer_[child]) == 0:
            self.recieved_ += 1
        self.evaluations_buffer_[child].append(msg)

        self.get_logger().info(f'Recieved evaluation from {child}. Total recieved: {self.recieved_} / {len(self.children_)}')
        if self.recieved_ == len(self.children_):
            # TODO: validate that all evaluations contain the same alternatives
            #  Currently this just uses the list from the last evaluation recieved.
            self.publish_evaluations_(msg.alternatives)
            self.recieved_ = 0

    def publish_evaluations_(self, alternatives):
        # Combine into a single evaluation and publish
        axes = []
        scores = []
        for child in self.children_:
            msg = self.evaluations_buffer_[child].pop(0)
            scores.append(scores_to_np_array(msg.scores, len(alternatives)))
            axes += msg.axes

        self.get_logger().info(f'Aggregating assessments of {axes} on'
                               f'alternatives {alternatives} with '
                               'policy: aggregate_multi_value_utility')
        self.pub_evaluation_.publish(
            Evaluation(alternatives=alternatives,
                       axes=axes,
                       scores=np.concatenate(scores, axis=1).flatten().tolist())
        )

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

        self.subs_ = [
                self.create_subscription(
                    Evaluation,
                    f'{child}/evaluation',
                    functools.partial(self.evaluations_cb, child),
                    10
                )
                for child in self.children_
        ]


        # Reset accumulator
        self.evaluations_buffer_ = {c : [] for c in children}
        self.recieved_ = 0

        print(self.children_)
        print(self.evaluations_buffer_)
        print(self.subs_)
        print(self.recieved_)

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
