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

import functools
from operator import attrgetter

import numpy as np


class LexJudgment:
    def __init__(self, alternative, scores, axies):
        """
        :param judgment: A :class:`decision_interfaces.msg.Judgment`
        :param axies: A list of :class:`string` axis names in tie-breaking order
            with most significant first.
        """
        self.features_ = scores
        self.rank = 0
        self.alternative = alternative
        self.axies_ = axies

    def __lt__(self, other):
        # Assume other is also a LexJudgment
        for axis in range(len(self.feaures_)):
            if self.features_[axis] < other.features_[axis]:
                return True
            if self.features_[axis] > other.features_[axis]:
                return False
        return False # They are equal


def lexicographical(judgments):
    """Rank each alternative by their score along each axis, breaking ties by
    the order of the axies.

    :param judgments: A set of :class:`LexJudgment` judgments for each considered alternative.

    :return: A pair of lists of alternatives and their corresponding integer ranks.
    """
    assert(len(judgments) > 0)
    judgments.sort()

    judgments[0].rank = 0
    i = 1
    while i < len(judgments):
        if judgments[i-1] < judgments[i]:
            judgments[i].rank = judgments[i-1].rank + 1
        else:
            judgments[i].rank = judgments[i-1].rank

    return [j.alternative for j in judgments], [j.rank for j in judgments]


def copeland_method(feature_matrix):
    """Rank each alternative by difference in the number of others that are worse
    than it and better than in in each feature.

    :param judgments: A set of :class:`decision_interfaces.msg.Judgment`
        judgments for each considered alternative.

    :return: A list of integer ranks matching the indicies of the feature matrix.
    """
    net_pref = net_preferences(feature_matrix)
    copeland = np.sum(np.sign(net_pref), axis=0)

    ranks = max(copeland) - copeland
    return ranks.astype(int).tolist()


def net_preferences(feature_matrix):
    """Calcualte the net preferences of a matrix of features.
    """
    n_alternatives = feature_matrix.shape[0]
    net_preferences = np.zeros((n_alternatives, n_alternatives))
    for i in range(n_alternatives):
        net_preferences[i,:] = np.sum(np.sign(feature_matrix[i,:] - feature_matrix), axis=0)
    return net_preferences


def create_feature_matrix(judgments):
    """Create a matrix of features from a set of judgments
    """

    # convert judgments into an A x F matix
    n_alternatives = len(judgments)
    n_features = len(judgments[0].features)
    feature_matrix = np.zeros((n_alternatives, n_features))
    
    for i, judgment in enumerate(judgments):
        feature_matrix[i,:] = [f.score for f in sorted(judgment.features, key=attrgetter('axis'))]

    return feature_matrix


def sequential_majority_comparison(feature_matrix):
    """Get single winning alternative by making pairwise comparisons of all alternatives.
    Note: this is essentially one iteration of swap sort

    :param judgments: A set of :class:`decision_interfaces.msg.Judgment`
        judgments for each considered alternative.

    :return: A pair of lists of alternatives and their corresponding integer ranks.
    """
    net_pref = net_preferences(feature_matrix)
    n_alternatives = feature_matrix.shape[0]
    winner = 0
    for i in range(n_alternatives):
        if net_pref[i,winner] > 0:
            winner = i
    ranks = [0] * len(n_alternatives)
    ranks[winner] = 1

    return ranks


def pareto_fronts(judgments):
    """Rank each alternative by the pareto front it belongs to.
    Adapted from https://github.com/iibrahimli/pareto_fronts/blob/master/pareto_fronts_dynamic.ipynb

    :param judgments: A set of :class:`decision_interfaces.msg.Judgment`
        judgments for each considered alternative.

    :return: A pair of lists of alternatives and their corresponding integer ranks.
    """
    feature_matrix = create_feature_matrix(judgments)
    n_alternatives = feature_matrix.shape[0]
    
    # holds lists of elements dominated by each element
    dominates = []
    fronts = [[]]
    n_dominators = [0] * n_alternatives

    # finding the first front
    for i in range(n_alternatives):
        dominates.append([])

        # TODO: use numpy array functions instead of for loop
        # dominates_matrix[i,:] = np.all(np.delete(feature_matrix, i, axis=0) < feature_matrix[i,:], axis=1)

        for j in range(n_alternatives):
            if i == j:
                continue
            # if i dominates j
            if np.all(feature_matrix[i,:] > feature_matrix[j,:]):
                dominates[i].append(j)
            # else if i is dominated by j
            elif np.all(feature_matrix[i,:] < feature_matrix[j,:]):
                n_dominators[i] += 1
        if n_dominators[i] == 0:
            fronts[0].append(i)
        
    # front number
    i = 0
    while len(fronts[i]) != 0:
        newfront = []
        for p in fronts[i]:
            for q in dominates[p]:
                n_dominators[q] -= 1
                if n_dominators[q] == 0:
                    newfront.append(q)
        fronts.append(newfront)
        i += 1

    # assign ranks
    alternatives = []
    ranks = []
    for f, front in enumerate(fronts):
        for i in front:
            alternatives.append(judgments[i].alternative)
            ranks.append(f)
    return alternatives, ranks

