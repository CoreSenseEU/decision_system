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

from decision_msgs.msg import WeakOrdering, OrderedEvaluation
from decision_msgs.action import Order
from abstract_decision_components.util import validate_matrix


class OrderActionServer(Node):
    """Orders alternatives based on how many others they dominate. Highest score is better.
    """

    def __init__(self, policy):
        self.policy_ = f'order_{policy}'
        self.policy_str = self.policy_ # mutable by children

        super().__init__(f'{self.policy_}_action_server')
        self.get_logger().info(f'Starting ORDER action server with policy: {self.policy_}')

        self.action_server_ = ActionServer(
                self,
                Order,
                '~/Order',
                self.order_cb)

        self.pub_ = self.create_publisher(
                OrderedEvaluation,
                'ordered_evaluation',
                10)

    def order_cb(self, goal_handle):
        evaluation = goal_handle.request.evaluation
        self.get_logger().info(f'Ordering {evaluation.alternatives} with policy: {self.policy_str}')

        try:
            validate_matrix(len(evaluation.alternatives), 
                            len(evaluation.axes), 
                            len(evaluation.scores))
            ranks = self.order(evaluation) # implemented by children
        except ValueError as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            return Order.Result()

        ordering = WeakOrdering(alternatives=evaluation.alternatives, ranks=ranks)
        self.pub_.publish(OrderedEvaluation(ordering=ordering, evaluation=evaluation))

        goal_handle.succeed()
        self.get_logger().info(f'Succeeded with ordering {ordering}')
        return Order.Result(ranks=ordering.ranks)

    # To be overridden by children
    def order(self, msg):
        raise NotImplementedError

