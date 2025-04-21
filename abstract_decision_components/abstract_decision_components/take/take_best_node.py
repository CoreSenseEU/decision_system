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

from decision_msgs.msg import Choice, OrderedEvaluation
from abstract_decision_components.take import take


class TakeBestNode(Node):
    def __init__(self):
        super().__init__('take_best_node')
        self.get_logger().info('Starting TAKE node with policy: take_best')

        self.declare_parameter('n', 0, Parameter.Type.INTEGER)
        self.declare_parameter('random_ties', False, Parameter.Type.BOOL)

        self.sub_ = self.create_subscription(
                OrderedEvaluation,
                'ordered_evaluation',
                self.ordered_evaluation_cb,
                10)
        self.pub_ = self.create_publisher(
                Choice,
                'choice',
                10)

    def ordered_evaluation_cb(self, msg):
        raise NotImplementedError("Not yet been tested")
        n = self.get_parameter('n').value
        ranked_alternatives = take.create_ordered_pairs(msg.ordering.alternatives, msg.ordering.ranks)

        choice = Choice(evaluation=msg.evaluation)
        if n == 0:
            choice.chosen = self.take_best(ranked_alternatives)
        elif self.get_parameter('random_ties').value:
            choice.chosen = self.take_n_random(ranked_alternatives)
        else:
            choice.chosen = self.take_n(ranked_alternatives)

        self.pub_.publish(choice)

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

    node = TakeBestNode()

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
