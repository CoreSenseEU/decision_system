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

def compare_size(choice, rhs, relation='='):
    """Accepts a ``choice`` relating the number of chosen alternatives to the right
    hand side, ``rhs``, with a binary ``relation``.

    :param choice: A set of chosen :class:`decision_interfaces.msg.Alternative`
        alternatives.
    :param rhs: A natural number to compare the size of ``choice`` to.
    :param relation: A relational operator with the rhs of ``choice`` on the
        right hand size and ``rhs`` on the left. Valid relation operators are:
            `<`, `>`, `<=`, `>=`, `=`, and `!=`. Treats invalid operators as =.
        Defaults to '='

    :raises ValueError: If ``relation`` is invald.

    :return: A :class:`boolean` indicating success or failure and a :class:`string`
        message indicating the reason.
    """
    match relation:
        case '>':
            success = len(choice) > rhs
        case '<':
            success = len(choice) < rhs
        case '>=':
            success = len(choice) >= rhs
        case '<=':
            success = len(choice) <= rhs
        case '=':
            success = len(choice) == rhs
        case '!=':
            success = len(choice) != rhs
        case _:
            raise ValueError(f"Recieved invalid relational operator: '{relation}'.")

    reason = f'test: {len(choice)} {relation} {rhs}?'
    return success, reason


def satisficing(choice, judgments, features):
    """Accepts a ``choice`` if the scores of all chosen alternatives are
    greater than or equal to the threshold values of each requested axis.

    :param choice: A set of chosen :class:`decision_interfaces.msg.Alternative`
        alternatives.
    :param judgments: A set of :class:`decision_interfaces.msg.Judgment`
        judgments for each considered alternative.
    :param features: A set of :class:`decision_interfaces.msg.Feature`
        features to compare all chosen alternatives to.

    :raises ValueError: If no judgment is found matching a chosen alternative.

    :return: A :class:`boolean` indicating success or failure and a :class:`string`
        message indicating the reason.
    """
    success = True
    reason = ''
    features_ = {f.axis : f.score for f in features}
    for chosen in choice:
        try:
            judgment = next(j for j in judgments if j.alternative.id == chosen.id)
        except StopIteration:
            raise ValueError(f'No judgment found for chosen alternative {chosen.id}')

        for feature in judgment.features:
            if feature.axis in features_ and feature.score < features_[feature.axis]:
                reason = f'{feature} of chosen {chosen} does not meet threshold {features_[feature.axis]}.'
                success = False
                break
        if not success:
            break

    return success, reason


def dominating(choice, judgments, axes=None):
    """
    Accepts a ``choice`` if the scores of all chosen alternatives are strictly
    greater than the best unchosen alternatives for each axis in ``axes``.

    :param choice: A set of chosen :class:`decision_interfaces.msg.Alternative`
        alternatives.
    :param judgments: A set of :class:`decision_interfaces.msg.Judgment`
        judgments for each considered alternative.
    :param axes: A set of :class:`string` axis names to check chosen alternatives
        for domination. Defaults to all axes

    :return: A :class:`boolean` indicating success or failure and a :class:`string`
        message indicating the reason.
    """
    best_scores_unchosen = { axis : float('-inf') for axis in axes }
    worst_scores_chosen = { axis : float('inf') for axis in axes }
    worst_chosen_by_axis = {}
    best_unchosen_by_axis = {}

    if axes is None and len(judgments) > 0:
        axes = {f.axis : f.score for f in judgments[0].features}

    for judgment in judgments:
        for feature in judgment.features:
            if feature.axis not in axes:
                continue
            if judgment.alternative in choice:
                worst_scores_chosen[feature.axis] = min(worst_scores_chosen[feature.axis], feature.score)
                worst_chosen_by_axis.update({feature.axis: judgment.alternative.id})
            else:
                best_scores_unchosen[feature.axis] = max(best_scores_unchosen[feature.axis], feature.score)
                best_unchosen_by_axis.update({feature.axis: judgment.alternative.id})

    success = True
    reason = ''
    for axis in axes:
        if worst_scores_chosen[axis] <= best_scores_unchosen[axis]:
            reason = f'Chosen Alternative({worst_chosen_by_axis[axis]})' \
                   + f' has Feature({axis},{worst_scores_chosen[axis]}) but it is dominated' \
                   + f' by unchosen Alternative({best_unchosen_by_axis[axis]})' \
                   + f' with Feature({axis},{best_scores_unchosen[axis]}).'
            success = False
            break

    return success, reason

