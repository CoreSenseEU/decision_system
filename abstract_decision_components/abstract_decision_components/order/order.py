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


def lexicographical(feature_matrix, index_array):
    """Rank each alternative by their score along each axis, breaking ties by
    the order of the axes.

    :param feature_matrix: An (N x M) numpy array of judgments where N is the
        number of alternatives and M is the number of features of each alternative.
    :param index_array: A numpy array of ranks of each feature in the feature matrix.

    :return: A list of integer ranks corresponding to the rows of the feature matrix.
    """
    args_sorted = np.argsort(feature_matrix[:, index_array], axis=0, kind='stable')[::-1,0]
    return [args_sorted.tolist().index(i) for i in range(feature_matrix.shape[0])]


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


def copeland_method(feature_matrix):
    """Rank each alternative by difference in the number of others that are worse
    than it and better than it in each feature.

    :param feature_matrix: An (N x M) numpy array of judgments where N is the
        number of alternatives and M is the number of features of each alternative.

    :return: A list of integer ranks corresponding to the rows of the feature matrix.
    """
    net_pref = _net_preferences(feature_matrix)
    copeland = np.sum(np.sign(net_pref), axis=1)

    ranks = max(copeland) - copeland
    return ranks.astype(int).tolist()


def sequential_majority_comparison(feature_matrix, confirming=False):
    """Get single winning alternative by making pairwise comparisons of all alternatives.
    Note: this is essentially one iteration of swap sort

    :param feature_matrix: An (N x M) numpy array of judgments where N is the
        number of alternatives and M is the number of features of each alternative.
    :param confirming: If True, use confirming (positive) dimensions only, else
        use the net preference. Defaults to False

    :return: A list of integer ranks corresponding to the rows of the feature matrix.
    """
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

    return ranks


def majority_rule(feature_matrix, strict=False):
    """Rank each alternative by the number of dominating features it has.

    :param feature_matrix: An (N x M) numpy array of judgments where N is the
        number of alternatives and M is the number of features of each alternative.
    :param strict: If true, assume axis features must be strictly greater than
        all others to be considered dominating. Defaults to False

    :return: A list of integer ranks corresponding to the rows of the feature matrix.
    """

    if strict:
        strict_cols = np.sum(feature_matrix == np.max(feature_matrix, axis=0), axis=0) == 1
        feature_matrix = feature_matrix[:,strict_cols]
    totals = np.sum(feature_matrix == np.max(feature_matrix, axis=0), axis=1)

    ranks = max(totals) - totals 
    return ranks


def pareto_fronts(feature_matrix, strict=False):
    """Rank each alternative by the pareto front it belongs to.
    Ref: https://github.com/iibrahimli/pareto_fronts/blob/master/pareto_fronts_dynamic.ipynb

    :param feature_matrix: An (N x M) numpy array of judgments where N is the
        number of alternatives and M is the number of features of each alternative.
    :param strict: If true, assume axis features must be strictly greater 
        to be considered dominating. Defaults to False

    :return: A list of integer ranks corresponding to the rows of the feature matrix.
    """
    n_alternatives = feature_matrix.shape[0]
    
    # holds lists of elements dominated by each element
    n_dominates = np.zeros(n_alternatives)

    for i in range(n_alternatives):
        if strict:
            n_dominates += np.all(feature_matrix > feature_matrix[i,:], axis=1)
        else:
            n_dominates += np.all(feature_matrix >= feature_matrix[i,:], axis=1)

    ranks = max(n_dominates) - n_dominates
    return ranks.astype(int).tolist()

