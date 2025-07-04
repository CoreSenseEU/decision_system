%%% Engines

%% UpdateCues
% iter_one - Use and lose one cue per round (dynamic)
%    reuse = False, iter_add = 1
% (used for eliminate-by-aspects)
engine(update_cues_iter_one).
has_ros_node(update_cues_iter_one, '/update_cues_prolog_action_server').
has_meta(update_cues_iter_one, 'behavior_trees/update_cues_structure.xml').
uses_ros_params(update_cues_iter_one, 'config/update_cues_iter_one.yaml').

% ...

%% UpdateAlternatives
% elim - assume all, eliminate until desired (diminishing)
%    keep_policy = taken, capacity = 0
% (used for eliminate-by-aspects)
engine(update_alternatives_elim).
has_ros_node(update_alternatives_elim, '/update_alternatives_prolog_action_server').
has_meta(update_alternatives_elim, 'behavior_trees/update_alternatives_structure.xml').
uses_ros_params(update_alternatives_elim, 'config/update_alternatives_elim.yaml').

% ...


%% Assess
% NOTE: No parameters.
engine(assess).
has_ros_node(assess, '/assess_action_server').
has_meta(assess, 'behavior_trees/assess_meta.xml').
uses_ros_params(assess, 'config/assess.yaml').

%% Aggregate
engine(aggregate_preferences).
has_ros_node(aggregate_preferences, '/aggregate_preferences_boolean_action_server').
has_meta(aggregate_preferences, 'behavior_trees/aggregate_meta.xml').
% NOTE: No parameters.
uses_ros_params(aggregate_preferences, 'config/aggregate_preferences.yaml').

% ...

%% Order
% TODO: rename this to indicate that the default uses the ording from evaluations
engine(order_lexicographical).
has_ros_node(order_lexicographical, '/order_lexicographical_action_server').
has_meta(order_lexicographical, 'behavior_trees/order_meta.xml').
% NOTE: no axis ordering!!
uses_ros_params(order_lexicographical, 'config/order_lexicographical_eval.yaml').

% ...

%% Take
% TODO: add constraint that with when no alternatives are being added, but there
%  is a maximum size constraint, `force_take` must be `True` otherwise we could run out of
%  alternatives if they are all tied.
engine(eliminate_worst).
has_ros_node(eliminate_worst, '/eliminate_worst_action_server').
has_meta(eliminate_worst, 'behavior_trees/take_meta.xml').
uses_ros_params(eliminate_worst, 'config/eliminate_worst.yaml').

%% Accept
% TODO: somehow indicate that the default size is `= 1`?
engine(accept_size).
has_ros_node(accept_size, '/accept_size_action_server').
has_meta(accept_size, 'behavior_trees/accept_meta.xml').
uses_ros_params(accept_size, 'config/accept_size_eq_1.yaml').

% ...

