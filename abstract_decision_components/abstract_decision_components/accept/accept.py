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
    :param rhs: An integer to compare the size of ``choice`` to.
    :param relation: A relational operator with the rhs of ``choice`` on the
        right hand size and ``rhs`` on the left. Valid relation operators are:
            `<`, `>`, `<=`, `>=`, `=`, and `!=`.
        Defaults to '='

    :raises ValueError: If ``relation`` is invald.

    :return: A :class:`boolean` indicating success or failure and a :class:`string`
        message indicating the reason (left blank upon success).
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

    reason = f'test: {len(choice)} ?{relation} {rhs}'
    return success, reason


def satisficing(chosen, judgments, features):
    """Accepts a ``chosen`` if the scores of all chosen alternatives are
    greater than or equal to the threshold values of each requested axis.

    :param chosen: A set of chosen :class:`decision_interfaces.msg.Alternative`
        alternatives.
    :param judgments: A set of :class:`decision_interfaces.msg.Judgment`
        judgments for each considered alternative.
    :param features: A set of tuples of axis names and threshold values 
        to compare all chosen alternatives to.

    :raises ValueError: If no judgment is found matching a chosen alternative 
        or a desired feature is missing from a judgment.

    :return: A :class:`boolean` indicating success or failure and a :class:`string`
        message indicating the reason (left blank upon success).
    """
    success = True
    reason = ''
    for alternative in chosen:
        try:
            judgment = next(j for j in judgments if j.alternative.id == alternative.id)
        except StopIteration:
            raise ValueError(f"Missing Judgment for chosen alternative '{alternative.id}'")

        judgment_features = {f.axis : f.score for f in judgment.features}
        for axis, threshold in features:
            if axis not in judgment_features:
                raise ValueError(f"Missing Judgment feature with axis '{axis}' for alternative alternative '{alternative.id}'")
            score = judgment_features[axis]
            if score < threshold:
                reason = f"Feature(axis={axis}, score={score}) of" \
                       + f" chosen alternative '{alternative.id}' does not meet" \
                       + f" the threshold value of {threshold}"
                success = False
                break
        if not success:
            break

    return success, reason


def dominating(chosen, judgments, axes=None):
    """
    Accepts a ``chosen`` if the scores of all chosen alternatives are strictly
    greater than the best unchosen alternatives for each axis in ``axes``.

    :param chosen: A set of chosen :class:`decision_interfaces.msg.Alternative`
        alternatives.
    :param judgments: A set of :class:`decision_interfaces.msg.Judgment`
        judgments for each considered alternative.
    :param axes: A set of :class:`string` axis names to check chosen alternatives
        for domination. Defaults to all axes

    :raises ValueError: If no judgment is found matching a chosen alternative 
        or a desired feature is missing from a judgment.

    :return: A :class:`boolean` indicating success or failure and a :class:`string`
        message indicating the reason (left blank upon success).
    """
    best_scores_unchosen = { axis : float('-inf') for axis in axes }
    worst_scores_chosen = { axis : float('inf') for axis in axes }
    worst_chosen_by_axis = {}
    best_unchosen_by_axis = {}

    if axes is None and len(judgments) > 0:
        axes = {f.axis : f.score for f in judgments[0].features}

    judgments_by_alternative = {j.alternative.id : j for j in judgments}
    for alternative in chosen:
        if alternative.id not in judgments_by_alternative.keys():
            raise ValueError(f"Missing Judgment for chosen alternative '{alternative.id}'")

    for judgment in judgments:
        judgment_features = {f.axis : f.score for f in judgment.features}
        for axis in axes:
            if axis not in judgment_features:
                raise ValueError(f"Missing Judgment feature with axis '{axis}' for chosen alternative '{judgment.alternative.id}'")

            if judgment.alternative in chosen:
                worst_scores_chosen[axis] = min(worst_scores_chosen[axis], judgment_features[axis])
                worst_chosen_by_axis.update({axis: judgment.alternative.id})
            else:
                best_scores_unchosen[axis] = max(best_scores_unchosen[axis], judgment_features[axis])
                best_unchosen_by_axis.update({axis: judgment.alternative.id})

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

