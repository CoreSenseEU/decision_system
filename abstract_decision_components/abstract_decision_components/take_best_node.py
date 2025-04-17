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
from operator import itemgetter
import random

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# from decision_interfaces.msg import Choice, WeakOrdering, Judgement
from decision_interfaces.srv import TakeN, TakeRelative
import take


class TakeBestNode(Node):
    def __init__(self):
        super().__init__('take_best_node')
        self.get_logger().info('Starting TAKE server with policy: take_best')

        # All services can be called in parallel
        cb_group = ReentrantCallbackGroup()

        self.srv_take_n = self.create_service(
            TakeN,
            'take_n',
            self.take_n_cb,
            callback_group=cb_group)

        self.srv_take_n_random = self.create_service(
            TakeN,
            'take_n_random',
            self.take_n_random_cb,
            callback_group=cb_group)

        self.srv_take_best = self.create_service(
            TakeRelative,
            'take_best',
            self.take_best_cb,
            callback_group=cb_group)


    def take_n_cb(self, request, response):
        """
        Takes the best N alternatives.
        """
        ranked_alternatives = take.create_ordered_pairs(request.ordering.alternatives, request.ordering.ranks)
        response.choice = take.take_best(ranked_alternatives, n=request.n)

        self.get_logger().info(f'{response.choice} taken with policy: take_n, n={request.n}')
        return response

    def take_n_random_cb(self, request, response):
        """
        Takes the best N alternatives, randomly selecting among tied classes.
        """
        ranked_alternatives = take.create_ordered_pairs(request.ordering.alternatives, request.ordering.ranks)
        response.choice = take.take_best(ranked_alternatives, n=request.n, random_ties=True)

        self.get_logger().info(f'{response.choice} taken with policy: take_n_random, n={request.n}')
        return response

    def take_best_cb(self, request, response):
        """
        Takes all the alternatives tied for best score.
        """
        ranked_alternatives = take.create_ordered_pairs(request.ordering.alternatives, request.ordering.ranks)
        response.choice = take.take_best(ranked_alternatives)

        self.get_logger().info(f'{response.choice} taken with policy: take_best')
        return response


def main(args=None):
    rclpy.init(args=args)

    node = TakeBestNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
