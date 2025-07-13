%%% World shared model for `room` and `objetcts` decisions

:- dynamic room/1.
:- dynamic object/1.

% Has a room or object been visited (assume we looked at all the objects there)
%
% visited(?RoomOrObject)
:- dynamic visited/1.


% These can now be used by rooms and objects
unlikely_if_visited(R, 0) :-
    (room(R); object(R)), visited(R).
unlikely_if_visited(R, 1) :-
    (room(R); object(R)), \+ visited(R).

% These can now be used by rooms and objects
neg_distance(R, S) :-
    (room(R); object(R)), random(N), S is -10 * N.
