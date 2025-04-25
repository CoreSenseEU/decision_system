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


class LexJudgment:
    def __init__(self, judgment, axes):
        """
        :param judgment: A :class:`decision_interfaces.msg.Judgment`
        :param axes: A list of :class:`string` axis names in tie-breaking order
            with most significant first.

        :raises ValueError: if all the set of axes in the judgment and the 
            ordering set are not equivalent. 
        """
        self.features_ = {f.axis : f.score for f in judgment.features}
        judgment_axes = sorted(self.features_.keys())
        if judgment_axes != sorted(axes):
            raise ValueError(f"Judgment axes {judgment_axes} of" \
                           + f" alternative '{judgment.alternative.id}' are" \
                           + f" incompatible with requested ordering {axes}")
        self.rank = 0
        self.alternative = judgment.alternative
        self.axes_ = axes


    def __lt__(self, other): # Higher score is lower, break ties by axies
        # Assume other is also a LexJudgment
        for axis in self.axes_:
            if self.features_[axis] > other.features_[axis]:
                return True
            if self.features_[axis] < other.features_[axis]:
                return False
        return False # they are equal

    def __repr__(self):
        return f'LexJudgment({self.alternative.id}, {str(self.features_)})'


# class LexJudgment:
#     def __init__(self, alternative, scores, axes):
#         """
#         :param judgment: A :class:`decision_interfaces.msg.Judgment`
#         :param axes: A list of :class:`string` axis names in tie-breaking order
#             with most significant first.
#         """
#         self.features_ = scores
#         self.rank = 0
#         self.alternative = alternative
#         self.axes_ = axes
#
#     def __lt__(self, other):
#         # Assume other is also a LexJudgment
#         for axis in range(len(self.feaures_)):
#             if self.features_[axis] < other.features_[axis]:
#                 return True
#             if self.features_[axis] > other.features_[axis]:
#                 return False
#         return False # They are equal


def lexicographical(judgments, axes):
    """Rank each alternative by their score along each axis, breaking ties by
    the order of the axes.

    :param judgments: A set of :class:`LexJudgment` judgments for each considered alternative.

    :return: A pair of lists of alternatives and their corresponding integer ranks.
    """
    lex_judgments = sorted([LexJudgment(j, axes) for j in judgments])
    print(lex_judgments)

    lex_judgments[0].rank = 0
    i = 1
    while i < len(lex_judgments):
        if lex_judgments[i-1] < lex_judgments[i]:
            lex_judgments[i].rank = lex_judgments[i-1].rank + 1
        else:
            lex_judgments[i].rank = lex_judgments[i-1].rank
        i += 1

    return [j.alternative for j in lex_judgments], [j.rank for j in lex_judgments]


# def inverse_borda(judgments):
#     """The naiive ordering of alternatives based on a single utility score.
#
#     :param judgments: A set of :class:`decision_interfaces.msg.Judgment`
#         judgments for each considered alternative.
#
#     :return: A list of integer ranks matching the indicies of the feature matrix.
#     """
#     assert(len(judgments[0].features) == 1)
#     return lexicographical(judgments, [judgments[0].features[0].axis])


def _create_feature_matrix(judgments):
    """Create a matrix of features from a set of judgments
    """
    # convert judgments into an A x F matix
    n_alternatives = len(judgments)
    n_features = len(judgments[0].features)
    feature_matrix = np.zeros((n_alternatives, n_features))
    
    for i, judgment in enumerate(judgments):
        # Assume all judgments have the same features in the same order
        feature_matrix[i,:] = [f.score for f in judgment.features]

    return feature_matrix


def _net_preferences(feature_matrix):
    """Calcualte the net preferences of a matrix of features.
    """
    n_alternatives = feature_matrix.shape[0]
    net_preferences = np.zeros((n_alternatives, n_alternatives))
    for i in range(n_alternatives):
        net_preferences[i,:] = np.sum(np.sign(feature_matrix[i,:] - feature_matrix), axis=1)
    return net_preferences


def _confirming_preferences(feature_matrix):
    """Calcualte the confirming preferences of a matrix of features.
    """
    n_alternatives = feature_matrix.shape[0]
    confirming_preferences = np.zeros((n_alternatives, n_alternatives))
    for i in range(n_alternatives):
        confirming_preferences[i,:] = np.sum((feature_matrix[i,:] - feature_matrix) > 0, axis=1)
    return confirming_preferences


def copeland_method(judgments):
    """Rank each alternative by difference in the number of others that are worse
    than it and better than it in each feature.

    :param judgments: A set of :class:`decision_interfaces.msg.Judgment`
        judgments for each considered alternative.

    :return: A pair of lists of alternatives and their corresponding integer ranks.
    """
    feature_matrix = _create_feature_matrix(judgments)
    net_pref = _net_preferences(feature_matrix)
    copeland = np.sum(np.sign(net_pref), axis=1)

    ranks = max(copeland) - copeland
    return [j.alternative for j in judgments], ranks.astype(int).tolist()


def sequential_majority_comparison(judgments, confirming=False):
    """Get single winning alternative by making pairwise comparisons of all alternatives.
    Note: this is essentially one iteration of swap sort

    :param judgments: A set of :class:`decision_interfaces.msg.Judgment`
        judgments for each considered alternative.
    :param confirming: If True, use confirming (positive) dimensions only, else
        use the net preference. Defaults to False

    :return: A pair of lists of alternatives and their corresponding integer ranks.
    """
    feature_matrix = _create_feature_matrix(judgments)
    if confirming:
        preferences = _confirming_preferences(feature_matrix)
    else:
        preferences = _net_preferences(feature_matrix)
    n_alternatives = feature_matrix.shape[0]
    winner = 0
    for i in range(n_alternatives):
        if preferences[i,winner] > 0:
            winner = i
    ranks = [1] * n_alternatives
    ranks[winner] = 0

    return [j.alternative for j in judgments], ranks


def majority_rule(judgments, strict=False):
    """Rank each alternative by the number of dominating features it has.

    :param judgments: A set of :class:`decision_interfaces.msg.Judgment`
        judgments for each considered alternative.
    :param strict: If true, assume axis features must be strictly greater than
        all others to be considered dominating. Defaults to False

    :return: A pair of lists of alternatives and their corresponding integer ranks.
    """
    feature_matrix = _create_feature_matrix(judgments)

    if strict:
        strict_cols = np.sum(feature_matrix == np.max(feature_matrix, axis=0), axis=0) == 1
        feature_matrix = feature_matrix[:,strict_cols]
    totals = np.sum(feature_matrix == np.max(feature_matrix, axis=0), axis=1)

    ranks = max(totals) - totals 
    return [j.alternative for j in judgments], ranks


def pareto_fronts(judgments, strict=False):
    """Rank each alternative by the pareto front it belongs to.
    Ref: https://github.com/iibrahimli/pareto_fronts/blob/master/pareto_fronts_dynamic.ipynb

    :param judgments: A set of :class:`decision_interfaces.msg.Judgment`
        judgments for each considered alternative.
    :param strict: If true, assume axis features must be strictly greater 
        to be considered dominating. Defaults to False

    :return: A pair of lists of alternatives and their corresponding integer ranks.
    """
    feature_matrix = _create_feature_matrix(judgments)
    n_alternatives = feature_matrix.shape[0]
    
    # holds lists of elements dominated by each element
    n_dominates = np.zeros(n_alternatives)

    for i in range(n_alternatives):
        if strict:
            n_dominates += np.all(feature_matrix > feature_matrix[i,:], axis=1)
        else:
            n_dominates += np.all(feature_matrix >= feature_matrix[i,:], axis=1)

    ranks = max(n_dominates) - n_dominates
    return [j.alternative for j in judgments], ranks.astype(int).tolist()

