%%% Requirements an agent may place on the acceptability of a choice

%% Types

% A requirement on a choice
%
% requirement(?Requirement)
:- dynamic requirement/1.
:- discontiguous requirement/1.



%% Individuals

% Restricts the choice to an absolute size.
% TODO: Right now this does not set the size property, just ensures that it
%   could be enforced and uses the default settings (N = 1).
%   This works for demonstration purposes with the understanding core but
%   would be much more useful if it was parameterized.
requirement(absolute_size_req).
