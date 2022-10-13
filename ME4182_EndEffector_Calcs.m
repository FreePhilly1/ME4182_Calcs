%% End Effector Tension Calculations
% ME 4182
% Fall 2022
% Team Snakes
clear; clc;
%% Parameters
m_endeffector_kg = 8;
v_max_ms = 2;
a_max_ms2 = 5;

frame_length_ft = 30;
frame_height_ft = frame_length_ft;

frame_length_m = frame_length_ft / 3.281;
frame_height_m = frame_height_ft / 3.281;

workspace_length_m = frame_length_m * 0.9;
workspace_height_m = frame_length_m * 0.9;

% TODO: GET TENSILE STRENGTH OF CABLES BEING USED
cable_tensile_strength = 1;

%% Define Workspace
force_weight_N = 9.81 * [0, -m_endeffector_kg];

% Max force occurs at top middle of workspace
offset_top_m = (frame_height_m - workspace_height_m) / 2;
offset_sides_m = frame_length_m / 2;
cable_angle_deg = atand(offset_top_m / offset_sides_m);


% sum(F) = 2 * T*sin(theta_cable) - mg = ma
max_tension_N = m_endeffector_kg * (a_max_ms2 + 9.81) / (2 * sind(cable_angle_deg));
fprintf("MAX TENSION IS %d", max_tension_N);