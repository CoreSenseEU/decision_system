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

from operator import itemgetter
import random


def create_ordered_pairs(alternatives, ranks):
    """Create a list of pairs of ``alternatives`` and their ``ranks`` ordered by rank.

    :param alternatives: A list of :class`decision_interfaces.msg.Alternative` 
        alternatives under considereation.
    :param ranks: A list of positive integer ranks, corresponding to each
        alternative.

    :return: A list of a list of pairs of ``alternatives`` and their ``ranks``
        ordered by rank.
    """
    return sorted(zip(alternatives, ranks), key = itemgetter(1))


def _choose_with_ties(ranked_alternatives, start):
    i = start
    j = start + 1
    tied_rank = ranked_alternatives[start][1]
    while i > 0 and ranked_alternatives[i-1][1] == tied_rank:
        i -= 1
    while j < len(ranked_alternatives) and ranked_alternatives[j-1][1] == tied_rank:
        j += 1

    choice = zip(*ranked_alternatives[:i]) + zip(*random.choices(ranked_alternatives[i:j], k=start-i+1))
    return choice


def take_best(ranked_alternatives, n=None, random_ties=False):
    """Takes the best alternatives.

    :param ranked_alternatives: A list of pairs of :class:`decision_interfaces.msg.Alternative`
        alternatives and integer ranks sorted by rank.
        under consideration.
    :param n: The number of best alternatives to return. Assume all are taken
        if this is larger than the number of ``alternatives``.
    :type n: int, optional
    :param random_ties: If true, randomly select between tied alternatives, default to False.
    :type random_ties: boolean, optional

    :return: A list of chosen alternatives.

    """
    if n is None:
        return _choose_with_ties(ranked_alternatives, 0)
    if random_ties:
        return _choose_with_ties(ranked_alternatives, n - 1)
    return zip(*ranked_alternatives[:n])


def eliminate_worst(ranked_alternatives, n, random_ties=False):
    """Eliminates the worst alternatives.

    :param ranked_alternatives: A list of pairs of :class:`decision_interfaces.msg.Alternative`
        alternatives and integer ranks sorted by rank.
        under consideration.
    :param n: The number of worst alternatives to remove.  Assume none are taken
        if this is larger than the number of ``alternatives``.
    :type n: int, optional
    :param random_ties: If true, randomly select between tied alternatives, default to False.
    :type random_ties: boolean, optional

    :return: A list of chosen alternatives.

    """
    if n is None: # Eliminate all tied for worst score
        return _choose_with_ties(ranked_alternatives, len(ranked_alternatives) - 1)
    if random_ties:
        return _choose_with_ties(ranked_alternatives, len(ranked_alternatives) - n)
    return zip(*ranked_alternatives[:-n])
