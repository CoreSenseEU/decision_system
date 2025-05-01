% Drop locations
:- dynamic drop/1.
drop(drop_1_dishwasher).
drop(drop_2_tableware).
drop(drop_3_livingroom).
drop(drop_4_toys).
drop(drop_5_general).
drop(drop_6_bedroom).
drop(drop_7_trash).

% Drop types
drop_type(dishwasher).
drop_type(tableware).
drop_type(livingroom).
drop_type(toys).
drop_type(general).
drop_type(bedroom).
drop_type(trash).

% Objects
object(obj_0_cup_clean).
object(obj_1_cup_dirty).
object(obj_3_fidget_spinner).
object(obj_4_book).
object(obj_7_bread).
object(obj_8_spoon_clean).
object(obj_9_spoon_dirty).
object(obj_10_tissue_box).
object(obj_11_beer).
object(obj_11_beer_empty).
object(obj_12_wood_cube).
object(obj_13_candle).

% Object Types
:- dynamic object_type/1.
object_type(cup).
object_type(fidget_spinner).
object_type(book).
object_type(bread).
object_type(spoon).
object_type(tissue_box).
object_type(beer).
object_type(wood_cube).
object_type(candle).

% Attributes
:- dynamic attribute/1.
attribute(clean).
attribute(dirty).
attribute(empty).

% Rooms
room(kitchen).
room(living_room).
room(office).
room(bedroom).

% Doorways
doorway(kitchen_to_bedroom).
doorway(kitchen_to_living).
doorway(bedroom_to_office).
doorway(office_to_living).

% Poses
:- dynamic pose/1.
pose(pose_drop_1_dishwasher).
pose(pose_drop_2_tableware).
pose(pose_drop_3_livingroom).
pose(pose_drop_4_toys).
pose(pose_drop_5_general).
pose(pose_drop_6_bedroom).
pose(pose_drop_7_trash).

% Extents
extent(extent_drop).

% 2D points
point_2d(point_2d_1_kitchen).
point_2d(point_2d_2_kitchen).
point_2d(point_2d_3_kitchen).
point_2d(point_2d_4_kitchen).
point_2d(point_2d_1_living_room).
point_2d(point_2d_2_living_room).
point_2d(point_2d_3_living_room).
point_2d(point_2d_4_living_room).
point_2d(point_2d_1_office).
point_2d(point_2d_2_office).
point_2d(point_2d_3_office).
point_2d(point_2d_4_office).
point_2d(point_2d_1_bedroom).
point_2d(point_2d_2_bedroom).
point_2d(point_2d_3_bedroom).
point_2d(point_2d_4_bedroom).
point_2d(point_2d_1_kitchen_to_bedroom).
point_2d(point_2d_2_kitchen_to_bedroom).
point_2d(point_2d_3_kitchen_to_bedroom).
point_2d(point_2d_4_kitchen_to_bedroom).
point_2d(point_2d_1_kitchen_to_living).
point_2d(point_2d_2_kitchen_to_living).
point_2d(point_2d_3_kitchen_to_living).
point_2d(point_2d_4_kitchen_to_living).
point_2d(point_2d_1_bedroom_to_office).
point_2d(point_2d_2_bedroom_to_office).
point_2d(point_2d_3_bedroom_to_office).
point_2d(point_2d_4_bedroom_to_office).
point_2d(point_2d_1_office_to_living).
point_2d(point_2d_2_office_to_living).
point_2d(point_2d_3_office_to_living).
point_2d(point_2d_4_office_to_living).


% --------------------------------
% ---- Object info
% --------------------------------

% Objects can be in a doorway
% in_doorway( <obj> , <doorway> ).
:- dynamic in_doorway/2.

% Objects have a pose
% has_pose( <obj>, <pose> ).
:- dynamic has_pose/2.
:- discontiguous has_pose/2.

% Objects can be held
% is_held( <obj> ).
:- dynamic is_held/1.

% Objects cannot have a pose if they are also held
% :- (
%     object(O), is_held(O) 
%     -> 
%     \+ has_pose(O,_)
%     ).


% Object have a drop type
% TODO: when a new object is found, how do we tell which one it is ... ?
% has_drop_type(obj_0_cup_clean, tableware).    %% Covered by logic
% has_drop_type(obj_1_cup_dirty, dishwasher).   %% Covered by logic
:- dynamic has_drop_type/2.
has_drop_type(obj_3_fidget_spinner, toys).
has_drop_type(obj_4_book, bedroom).
has_drop_type(obj_7_bread, trash).
% has_drop_type(obj_8_spoon_clean, tableware).  %% Covered by logic
% has_drop_type(obj_9_spoon_dirty, dishwasher). %% Covered by logic
has_drop_type(obj_10_tissue_box, livingroom).
has_drop_type(obj_11_beer, tableware).
% has_drop_type(obj_11_beer_empty, trash).      %% Covered by logic
has_drop_type(obj_12_wood_cube, bedroom).
has_drop_type(obj_13_candle, livingroom).

% Clean tableware goes on the table
has_drop_type(O, tableware) :- 
  has_attribute(O, clean),
  has_type(O, T),
  is_tableware(T).

% Dirty tableware goes in the dishwasher
has_drop_type(O, dishwasher) :-
  has_attribute(O, dirty),
  has_type(O, T),
  is_tableware(T).

% Empty beers go in the trash
has_drop_type(O, trash) :-
  has_attribute(O, empty),
  has_type(O, beer).

% Objects have an object_type
:- dynamic has_type/2.
has_type(obj_0_cup_clean, cup).
has_type(obj_1_cup_dirty, cup).
has_type(obj_3_fidget_spinner, fidget_spinner).
has_type(obj_4_book, book).
has_type(obj_7_bread, bread).
has_type(obj_8_spoon_clean, spoon).
has_type(obj_9_spoon_dirty, spoon).
has_type(obj_10_tissue_box, tissue_box).
has_type(obj_11_beer, beer).
has_type(obj_11_beer_empty, beer).
has_type(obj_12_wood_cube, wood_cube).
has_type(obj_13_candle, candle).

% Some objecs are slow to drop
slow_drop(obj_3_fidget_spinner).
slow_drop(obj_3_candle).


% --------------------------------
% ---- Object Type info
% --------------------------------

% Object_types can be tableware
is_tableware(cup).
is_tableware(spoon).



% --------------------------------
% ---- Room info
% --------------------------------

% Rooms have vertices (not necessarily square)
has_corners(kitchen, [point_2d_1_kitchen, point_2d_2_kitchen, point_2d_3_kitchen, point_2d_4_kitchen]).
has_corners(living_room, [point_2d_1_living_room, point_2d_2_living_room, point_2d_3_living_room, point_2d_4_living_room]).
has_corners(office, [point_2d_1_office, point_2d_2_office, point_2d_3_office, point_2d_4_office]).
has_corners(bedroom, [point_2d_1_bedroom, point_2d_2_bedroom, point_2d_3_bedroom, point_2d_4_bedroom]).


% --------------------------------
% ---- Doorway info
% --------------------------------

% Doors have vertices (not necessarily square)
has_corners(kitchen_to_bedroom, [point_2d_1_kitchen_to_bedroom, point_2d_2_kitchen_to_bedroom, point_2d_3_kitchen_to_bedroom, point_2d_4_kitchen_to_bedroom]).
has_corners(kitchen_to_living, [point_2d_1_kitchen_to_living, point_2d_2_kitchen_to_living, point_2d_3_kitchen_to_living, point_2d_4_kitchen_to_living]).
has_corners(bedroom_to_office, [point_2d_1_bedroom_to_office, point_2d_2_bedroom_to_office, point_2d_3_bedroom_to_office, point_2d_4_bedroom_to_office]).
has_corners(office_to_living, [point_2d_1_office_to_living, point_2d_2_office_to_living, point_2d_3_office_to_living, point_2d_4_office_to_living]).


% Doors connect two rooms
connects_rooms(kitchen_to_bedroom, kitchen, bedroom).
connects_rooms(kitchen_to_living, kitchen, living_room).
connects_rooms(bedroom_to_office, bedroom, office).
connects_rooms(office_to_living, office, living_room).
% Connecting rooms is reflexive
connects_rooms(D, R1, R2) :- conects_rooms(D, R2, R1).



% --------------------------------
% ---- Drop location info
% --------------------------------

% Drop locations have a pose
has_pose(drop_1_dishwasher, pose_drop_1_dishwashwer).
has_pose(drop_2_tableware,  pose_drop_2_tableware).
has_pose(drop_3_livingroom, pose_drop_3_livingroom).
has_pose(drop_4_toys,       pose_drop_4_toys).
has_pose(drop_5_general,    pose_drop_5_general).
has_pose(drop_6_bedroom,    pose_drop_6_bedroom).
has_pose(drop_7_trash,      pose_drop_7_trash).

% Drop locations have an extent which is centered at the pose
has_extent(D, extent_drop) :- drop(D).

% Drop locations have a drop_type
:- discontiguous has_drop_type/2.
has_drop_type(drop_1_dishwasher, dishwasher).
has_drop_type(drop_2_tableware, tableware).
has_drop_type(drop_3_livingroom, livingroom).
has_drop_type(drop_4_toys, toys).
has_drop_type(drop_5_general, general).
has_drop_type(drop_6_bedroom, bedroom).
has_drop_type(drop_7_trash, trash).


% --------------------------------
% ---- Pose info
% --------------------------------

% Poses have 7D coordinates X,Y,Z, quaterion    [meters, _]
:- dynamic has_coordinates_7d/8.
has_coordinates_7d(pose_drop_1_dishwasher, 1.5, 0, 0, 0, 0, 0, 0).
has_coordinates_7d(pose_drop_2_tableware, 0.356888, -1.47612, 0, 0, 0, 0, 0).
has_coordinates_7d(pose_drop_3_livingroom, -4, -3, 0, 0, 0, 0, 0).
has_coordinates_7d(pose_drop_4_toys, -8, 3, 0, 0, 0, 0, 0).
has_coordinates_7d(pose_drop_5_general, -2.5096, 6.78965, 0, 0, 0, 0, 0).
has_coordinates_7d(pose_drop_6_bedroom, 2.15269, 6.51919, 0, 0, 0, 0, 0).
has_coordinates_7d(pose_drop_7_trash, -8.64577, 1.30639, 0, 0, 0, 0, 0).



% --------------------------------
% ---- Extent info
% --------------------------------

% Extents have 3D dimensions X,Y,Z [meters]
has_dimensions(extent_door, 1, 1, 0.01).



% --------------------------------
% ---- Point info
% --------------------------------

% Points2D have coordinates X,Y
has_coordinates(point_2d_1_kitchen, -3.06, 2.04).
has_coordinates(point_2d_2_kitchen, 2.71, 1.95).
has_coordinates(point_2d_3_kitchen, 2.77, -1.94).
has_coordinates(point_2d_4_kitchen, -3.07, -1.92).
has_coordinates(point_2d_1_living_room, -3.15, -4.84).
has_coordinates(point_2d_2_living_room, -9.56, -4.79).
has_coordinates(point_2d_3_living_room, -9.36, 1.88).
has_coordinates(point_2d_4_living_room, -3.25, 1.81).
has_coordinates(point_2d_1_office, -3.25, 2.04).
has_coordinates(point_2d_2_office, -9.36, 2.11).
has_coordinates(point_2d_3_office, -9.35, 7.37).
has_coordinates(point_2d_4_office, -3.24, 7.37).
has_coordinates(point_2d_1_bedroom, 2.76, 2.18).
has_coordinates(point_2d_2_bedroom, -3.02, 2.05).
has_coordinates(point_2d_3_bedroom, -2.99, 7.25).
has_coordinates(point_2d_4_bedroom, 2.76, 7.38).
has_coordinates(point_2d_1_kitchen_to_bedroom, -3.01, 1.84).
has_coordinates(point_2d_2_kitchen_to_bedroom, -3.01, 2.36).
has_coordinates(point_2d_3_kitchen_to_bedroom, -1.58, 2.36).
has_coordinates(point_2d_4_kitchen_to_bedroom, -1.58, 1.84).
has_coordinates(point_2d_1_kitchen_to_living, -3.46, 0.63).
has_coordinates(point_2d_2_kitchen_to_living, -2.70, 0.63).
has_coordinates(point_2d_3_kitchen_to_living, -2.70, -0.74).
has_coordinates(point_2d_4_kitchen_to_living, -3.46, -0.74).
has_coordinates(point_2d_1_bedroom_to_office, -2.73, 3.47).
has_coordinates(point_2d_2_bedroom_to_office, -3.42, 3.52).
has_coordinates(point_2d_3_bedroom_to_office, -3.42, 4.86).
has_coordinates(point_2d_4_bedroom_to_office, -2.73, 4.86).
has_coordinates(point_2d_1_office_to_living, -5.06, 2.61).
has_coordinates(point_2d_2_office_to_living, -5.06, 1.53).
has_coordinates(point_2d_3_office_to_living, -7.24, 1.53).
has_coordinates(point_2d_4_office_to_living, -7.24, 2.61).


