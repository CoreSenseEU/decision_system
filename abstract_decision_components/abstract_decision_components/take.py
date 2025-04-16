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

# from decision_interfaces.msg import Choice, WeakOrdering, Judgement
from decision_interfaces.srv import TakeN, TakeRelative


class TakeNode(Node):
    def __init__(self):
        super().__init__('take_server')
        self.get_logger().info('Starting TAKE servers')

        # All services can be called in parallel
        cb_group = ReentrantCallbackGroup()

        self.srv_take_n = self.create_service(
            TakeN,
            'take_n',
            self.take_n_cb,
            callback_group=cb_group)

        self.srv_eliminate_n = self.create_service(
            TakeN,
            'eliminate_n',
            self.eliminate_n_cb,
            callback_group=cb_group)

        self.srv_take_n_random = self.create_service(
            TakeN,
            'take_n_random',
            self.take_n_random_cb,
            callback_group=cb_group)

        self.srv_eliminate_n_random = self.create_service(
            TakeN,
            'eliminate_n_random',
            self.eliminate_n_random_cb,
            callback_group=cb_group)

        self.srv_take_best = self.create_service(
            TakeRelative,
            'take_best',
            self.take_best_cb,
            callback_group=cb_group)

        self.srv_eliminate_worst = self.create_service(
            TakeRelative,
            'eliminate_worst',
            self.eliminate_worst_cb,
            callback_group=cb_group)
    
    def take_n_cb(self, request, response):
        """
        Takes the best N alternatives.
        """
        # Assume all are taken if n is too big
        if request.n >= len(request.ordering.alternatives):
            response.choice = request.ordering.alternatives
            return response

        sorted_alternatives = sorted(
                zip(request.ordering.alternatives, request.ordering.ranks), 
                key = itemgetter(1),
            )
        response.choice = [a for a, _ in sorted_alternatives[:request.n]]

        self.get_logger().info(f'{response.choice} taken with policy: take_n, n={request.n}')
        return response

    def emilinate_n_cb(self, request, response):
        """
        Eliminates the worst N alternatives.
        """
        # Assume all are eliminated if n is too big
        if request.n >= len(request.ordering.alternatives):
            response.choice = []
            return response

        sorted_alternatives = sorted(
                zip(request.ordering.alternatives, request.ordering.ranks), 
                key = itemgetter(1),
            )
        response.choice = [a for a, _ in sorted_alternatives[:len(sorted_alternatives) - request.n]]

        self.get_logger().info(f'{response.choice} taken with policy: eliminate_n, n={request.n}')
        return response

    def take_n_random_cb(self, request, response):
        """
        Takes the best N alternatives, randomly selecting among tied classes.
        """
        # Assume all are taken if n is too big
        if request.n >= len(request.ordering.alternatives):
            response.choice = request.ordering.alternatives
            return response

        sorted_alternatives = sorted(
                zip(request.ordering.alternatives, request.ordering.ranks), 
                key = itemgetter(1),
            )

        i = 0
        j = response.n - 1
        worst_rank = sorted_alternatives[j-1][1]
        while i < j and sorted_alternatives[i][1] < worst_rank:
            i += 1
        while j < len(sorted_alternatives) and sorted_alternatives[j][1] == worst_rank:
            j += 1

        response.choice = [a for a, _ in sorted_alternatives[:i]] + [a for a,_ in random.choices(sorted_alternatives[i:j], k=response.n-i)]

        self.get_logger().info(f'{response.choice} taken with policy: take_n_random, n={request.n}')
        return response

    def eliminate_n_random_cb(self, request, response):
        """
        Eliminates the worst N alternatives, randomly selecting among tied classes.
        """
        # Assume all are eliminated if n is too big
        if request.n >= len(request.ordering.alternatives):
            response.choice = []
            return response

        sorted_alternatives = sorted(
                zip(request.ordering.alternatives, request.ordering.ranks), 
                key = itemgetter(1),
            )

        i = len(sorted_alternatives) - response.n
        j = i + 1
        best_eliminated_rank = sorted_alternatives[i][1]
        while i > 0 and sorted_alternatives[i-1][1] == best_eliminated_rank:
            i -= 1
        while j < len(sorted_alternatives) and sorted_alternatives[j-1][1] == best_eliminated_rank:
            j += 1

        response.choice = [a for a, _ in sorted_alternatives[:i]] + [a for a,_ in random.choices(sorted_alternatives[i:j], k=response.n-i)]

        self.get_logger().info(f'{response.choice} taken with policy: eliminate_n_random, n={request.n}')
        return response

    def take_best_cb(self, request, response):
        """
        Takes all the alternatives tied for best score.
        """
        # Ignore empty
        if len(request.ordering.alternatives) == 0:
            response.choice = []
            return response

        sorted_alternatives = sorted(
                zip(request.ordering.alternatives, request.ordering.ranks), 
                key = itemgetter(1),
            )

        i = 0
        best_rank = sorted_alternatives[i][1]
        while i < len(sorted_alternatives) and sorted_alternatives[i][1] == best_rank:
            i += 1

        response.choice = [a for a, _ in sorted_alternatives[:i]]
        self.get_logger().info(f'{response.choice} taken with policy: take_best')
        return response

    def eliminate_worst_cb(self, request, response):
        """
        Eliminates all the alternatives tied for worst score.
        """
        # Ignore empty
        if len(request.ordering.alternatives) == 0:
            response.choice = []
            return response

        sorted_alternatives = sorted(
                zip(request.ordering.alternatives, request.ordering.ranks), 
                key = itemgetter(1),
            )

        i = len(sorted_alternatives) - 1
        worst_rank = sorted_alternatives[i][1]
        while i > 0 and sorted_alternatives[i][1] == worst_rank:
            i -= 1

        response.choice = [a for a, _ in sorted_alternatives[:i+1]]
        self.get_logger().info(f'{response.choice} taken with policy: eliminate_worst')
        return response


def main(args=None):
    rclpy.init(args=args)

    take_server = TakeNode()
    try:
        rclpy.spin(take_server)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        take_server.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
