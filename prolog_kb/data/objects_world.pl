%%% World model for `objects` decision

% Assume none of these are tidied objects
object(object_1).
object(object_2).
object(object_3).
object(object_4).
object(object_5).
object(object_6).

% Has an object been visited already?
%
% visited(?Room)
:- dynamic visited/1.

% Object in doorway
:- dynamic in_doorway/1.

object_unfavorable_if_in_doorway(O, -1) :-
    object(O), in_doorway(O).
object_unfavorable_if_in_doorway(O, 1) :-
    object(O), \+ in_doorway(O).


% Confidence object is book
:- dynamic confidence_is_book/2.

object_favorable_if_confidence_is_book(O, -2.0) :-
    object(O), confidence_is_book(O, 0.0).  % Object is certainly not the book
object_favorable_if_confidence_is_book(O, S) :-
    object(O), confidence_is_book(O, C), C < 0.5, S is -1 + C. % Object is probably not the book
object_favorable_if_confidence_is_book(O, C) :-
    object(O), confidence_is_book(O, C), C >= 0.5. % Object probably is the book (or is certainly the book)
object_favorable_if_confidence_is_book(O, 0.0) :- %% Possibility of Object being the book is unknown
    object(O), \+ confidence_is_book(O, _).
