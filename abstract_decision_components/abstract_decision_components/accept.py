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
from rclpy.callback_groups import ReentrantCallbackGroup

from decision_interfaces.srv import AcceptChoice, AcceptRelational, \
    AcceptSatisficing, AcceptDominating


class AcceptNode(Node):
    def __init__(self):
        super().__init__('accept_server')
        self.get_logger().info('Starting ACCEPT servers')

        # All services can be called in parallel
        cb_group = ReentrantCallbackGroup()

        self.srv_always = self.create_service(
            AcceptChoice,
            'accept_always',
            self.always_accept_cb,
            callback_group=cb_group)

        self.srv_n_relational = self.create_service(
            AcceptRelational,
            'accept_n_relational',
            self.accept_n_relational_cb,
            callback_group=cb_group)

        self.srv_one_satisficing = self.create_service(
            AcceptSatisficing,
            'accept_one_satisficing',
            self.accept_one_satisficing_cb,
            callback_group=cb_group)

        self.srv_all_satisficing = self.create_service(
            AcceptSatisficing,
            'accept_all_satisficing',
            self.accept_all_satisficing_cb,
            callback_group=cb_group)

        self.srv_dominating = self.create_service(
            AcceptDominating,
            'accept_dominating',
            self.accept_dominating_cb,
            callback_group=cb_group)
    
    def always_accept_cb(self, request, response):
        """
        Always accepts the choice no matter what it is.
        """
        self.get_logger().info(f'Accepting choice {request.choice} with policy: accept_always')
        response.success = True
        return response

    def accept_n_relational_cb(self, request, response):
        """
        Accepts a choice based on the number of chosen alternatives.

        The number of chosen alternatives is compared to the threshold value (n)
        using a relational operator (op) with n on the right hand side.
        Valid operators are: <, >, <=, >=, =, and !=. Treats invalid operators
        as =.
        """
        match request.op:
            case '>':
                response.success = len(request.choice) > request.n
            case '<':
                response.success = len(request.choice) < request.n
            case '>=':
                response.success = len(request.choice) >= request.n
            case '<=':
                response.success = len(request.choice) <= request.n
            case '=':
                response.success = len(request.choice) == request.n
            case '!=':
                response.success = len(request.choice) != request.n
            case _:
                self.get_logger().warn(f"Recieved invalid relational operator: '{request.op}'. Defaulting to '='.")
                response.success = len(request.choice) == request.n

        response.result = f'test: {len(request.choice)} {request.op} {request.n}?'
        if response.success:
            verb = 'Accepting'
        else:
            verb = 'Rejecting'
        self.get_logger().info(f'{verb} choice {request.choice} with policy: accept_n_relational, "len(choice) {request.op} {request.n}"')
        return response

    def accept_one_satisficing_cb(self, request, response):
        """
        Accepts a choice containing only one alternative which scores are
        greater than or equal to the threshold values of each requested axis.
        """
        response.success = True
        if len(request.choice) == 1:
            judgment = next(j for j in request.judgments if j.alternative.id == request.choice[0].id)
            features_ = {f.axis : f.score for f in request.features}

            for feature in judgment.features:
                if feature.axis in features_ and feature.score < features_[feature.axis]:
                    response.result = f'Feature(axis={feature.axis},score={feature.score}) does not meet threshold {features_[feature.axis]}.'
                    response.success = False
                    break
        else:
            response.result = f'len(choice)={len(request.choice)} must be 1'
            response.success = False

        if response.success:
            verb = 'Accepting'
        else:
            verb = 'Rejecting'
        self.get_logger().info(f'{verb} choice {request.choice} with policy: accept_one_satisficing with thresholds {features_}')
        return response

    def accept_all_satisficing_cb(self, request, response):
        """
        Accepts a choice if the scores of all chosen alternatives are
        greater than or equal to the threshold values of each requested axis.
        """
        response.success = True
        features_ = {f.axis : f.score for f in request.features}
        for chosen in request.choice:
            judgment = next(j for j in request.judgments if j.alternative.id == chosen.id)

            for feature in judgment.features:
                if feature.axis in features_ and feature.score < features_[feature.axis]:
                    response.result = f'{feature} of chosen {chosen} does not meet threshold {features_[feature.axis]}.'
                    response.success = False
                    break
            if not response.success:
                break

        if response.success:
            verb = 'Accepting'
        else:
            verb = 'Rejecting'
        self.get_logger().info(f'{verb} choice {request.choice} with policy: accept_all_satisficing, "{request.features}"')
        return response

    def accept_dominating_cb(self, request, response):
        """
        Accepts a choice if the scores of all chosen alternatives are strictly
        greater than the best unchosen alternative for each requested axis.
        """
        best_scores_unchosen = { axis : float('-inf') for axis in request.axies }
        worst_scores_chosen = { axis : float('inf') for axis in request.axies }

        for judgment in request.judgments:
            for feature in judgment.features:
                if feature.axis not in request.axies:
                    continue
                if judgment.alternative in request.choice:
                    worst_scores_chosen[feature.axis] = min(worst_scores_chosen[feature.axis], feature.score)
                else:
                    best_scores_unchosen[feature.axis] = max(best_scores_unchosen[feature.axis], feature.score)

        response.success = True
        for axis in request.axies:
            if worst_scores_chosen[axis] <= best_scores_unchosen[axis]:
                response.result = f'A chosen alternative has Feature({axis},{worst_scores_chosen[axis]}) but it is dominated by {best_scores_unchosen[axis]}.'
                response.success = False
                break

        if response.success:
            verb = 'Accepting'
        else:
            verb = 'Rejecting'
        self.get_logger().info(f'{verb} choice {request.choice} with policy: accept_dominating, "{request.axies}"')
        return response


def main(args=None):
    rclpy.init(args=args)

    accept_server = AcceptNode()
    try:
        rclpy.spin(accept_server)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        accept_server.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
