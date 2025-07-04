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

import numpy as np


def compare_size(choice, rhs, relation='='):
    """Accepts a ``choice`` relating the number of chosen alternatives to the right
    hand side, ``rhs``, with a binary ``relation``.

    :param choice: A set of chosen :class:`decision_interfaces.msg.Alternative`
        alternatives.
    :param rhs: An integer to compare the size of ``choice`` to.
    :param relation: A relational operator with the rhs of ``choice`` on the
        right hand size and ``rhs`` on the left. Valid relation operators are:
            `lt`, `gt`, `leq`, `geq`, `eq`, and `neq`.
        Defaults to 'eq'

    :raises ValueError: If ``relation`` is invald.

    :return: A :class:`boolean` indicating success or failure and a :class:`string`
        message indicating the reason (left blank upon success).
    """
    match relation:
        case 'lt':
            return len(choice) > rhs
        case 'gt':
            return len(choice) < rhs
        case 'leq':
            return len(choice) >= rhs
        case 'geq':
            return len(choice) <= rhs
        case 'eq':
            return len(choice) == rhs
        case 'neq':
            return len(choice) != rhs

    # TODO: move this to parameter checking in AcceptSizeNode?
    raise ValueError(f"Recieved invalid relational operator: '{relation}'.")


def satisficing(scores, features):
    """Accepts if the feature scores of all chosen alternatives are
    greater than or equal to the threshold values of each requested axis.

    :param scores: An numpy array of shape (n_chosen, n_features) of feature
        scores for the chosen alternatives.
    :param features: A numpy array of threshold values to compare all chosen
        alternatives to.

    :return: A :class:`boolean` indicating success or failure.
    """
    return bool(np.all(scores > features))


def dominating(chosen_indices, scores):
    """
    Accepts if the scores of all chosen alternatives are strictly greater than
    the best unchosen alternatives.

    :param chosen_indices: A set of indices of chosen :class:`decision_interfaces.msg.Alternative`
        alternatives.
    :param scores: An numpy array of shape (n_chosen, n_features) of feature
        scores for the chosen alternatives.

    :raises ValueError: If no judgment is found matching a chosen alternative 
        or a desired feature is missing from a judgment.

    :return: A :class:`boolean` indicating success or failure and a :class:`string`
        message indicating the reason (left blank upon success).
    """
    # Everything was chosen
    if scores.shape[0] == len(chosen_indices):
        return True

    chosen_scores = scores[np.array(chosen_indices),:]
    best_unchosen_scores = np.max(np.delete(scores, chosen_indices, axis=0), axis=0)

    return bool(np.all(chosen_scores > best_unchosen_scores))

