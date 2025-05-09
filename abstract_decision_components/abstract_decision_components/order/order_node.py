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

from decision_msgs.msg import Evaluation, OrderedEvaluation, WeakOrdering
from abstract_decision_components.util import validate_matrix


class OrderNode(Node):
    """Orders alternatives based on how many others they dominate. Highest score is better.
    """

    def __init__(self, policy):
        self.policy_ = f'order_{policy}'
        self.policy_str = self.policy_ # mutable by children

        super().__init__(f'{self.policy_}_node')
        self.get_logger().info(f'Starting ORDER node with policy: {self.policy_}')

        self.sub_ = self.create_subscription(
                Evaluation,
                'evaluation',
                self.evaluation_cb,
                10)

        self.pub_ = self.create_publisher(
                OrderedEvaluation,
                'ordered_evaluation',
                10)

    def evaluation_cb(self, msg):
        self.get_logger().info(f'Ordering {msg.alternatives} with policy: {self.policy_str}')

        try:
            validate_matrix(len(msg.alternatives), len(msg.axes), len(msg.scores))
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        try:
            ranks = self.order(msg) # implemented by children
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        ordering = WeakOrdering(alternatives=msg.alternatives, ranks=ranks)
        self.pub_.publish(OrderedEvaluation(ordering=ordering, evaluation=msg))

    # To be overridden by children
    def order(self, msg):
        raise NotImplementedError

