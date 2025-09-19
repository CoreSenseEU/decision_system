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

from decision_msgs.msg import AlternativeArray
from decision_msgs.action import Take
from abstract_decision_components.take import take
from abstract_decision_components.util import load_cs_description


class TakeBestActionServer(Node):
    """Take the best alternatives according to the policy.

    :param n: An integer number of best alternatives to take. Takes all 
        alternatives tied for best if this is less than `1`. Defaults to 0
    :param random_ties: If `True`, break ties randomly, default to False
    """
    def __init__(self):
        self.policy_ = 'take_best'
        super().__init__('take_best_action_server')
        self.get_logger().info('Starting TAKE action server with policy: take_best')

        self.declare_parameter('coresense_engine', load_cs_description(self.policy_))

        self.declare_parameter('n', 0)
        self.declare_parameter('random_ties', False)

        self.action_server_ = ActionServer(
                self,
                Take,
                '~/Take',
                self.take_cb)

        self.pub_ = self.create_publisher(
                AlternativeArray,
                'choice',
                10)

    def take_cb(self, goal_handle):
        ordering = goal_handle.request.ordering
        self.get_logger().info(f'Taking from {ordering.alternatives} with policy: take_best')

        n = self.get_parameter('n').value
        try:
            ranked_alternatives = take.create_ordered_pairs(ordering.alternatives, ordering.ranks)
        except ValueError as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            return Take.Result()

        if n < 1:
            chosen = self.take_best(ranked_alternatives)
        elif self.get_parameter('random_ties').value:
            chosen = self.take_n_random(ranked_alternatives, n)
        else:
            chosen = self.take_n(ranked_alternatives, n)

        choice = AlternativeArray(alternatives=chosen)
        self.pub_.publish(choice)

        goal_handle.succeed()
        self.get_logger().info(f'Succeeded with choice {choice}')
        return Take.Result(chosen=choice.alternatives)

    def take_n(self, ranked_alternatives, n):
        """
        Takes the best N alternatives.
        """
        chosen = take.take_best(ranked_alternatives, n=n)

        self.get_logger().info(f'{chosen} taken with policy: take_n, n={n}')
        return chosen

    def take_n_random(self, ranked_alternatives, n):
        """
        Takes the best N alternatives, randomly selecting among tied classes.
        """
        chosen = take.take_best(ranked_alternatives, n=n, random_ties=True)

        self.get_logger().info(f'{chosen} taken with policy: take_n_random, n={n}')
        return chosen

    def take_best(self, ranked_alternatives):
        """
        Takes all the alternatives tied for best score.
        """
        chosen = take.take_best(ranked_alternatives)

        self.get_logger().info(f'{chosen} taken with policy: take_best')
        return chosen


def main(args=None):
    rclpy.init(args=args)

    node = TakeBestActionServer()

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
