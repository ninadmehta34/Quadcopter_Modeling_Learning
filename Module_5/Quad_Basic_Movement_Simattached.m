%% Module 5: Quadcopter Movement - Simulation
% =========================================================================
% Author: Ninad Mehta
% Date: 2026-11-12
% Source: MathWorks "Introduction to Quadcopter Modeling & Simulation"
%         Module 5 - Quadcopter Movement
% =========================================================================

clear quad_m W w
% changing the below lines to look for data file manually
%load x_quad_params.mat quad_m W w
% Construct the full path to the data file
rootPath = fullfile(pwd);
fileloc = fullfile(rootPath, 'Data', 'x_quad_params.mat');


if ~exist(fileloc, 'file')
    error('Parameter file not found: %s', fileloc);
end

% Parameters
load(fileloc, 'quad_m', 'W', 'w');


% Input
w = [3980 3980 4010 4010]; 


modelName = 'UAV_Sim_Mod';
open_system(modelName);


% Simulation Time = 10 seconds
set_param(modelName, 'StopTime', '10');


% Run the Simulation
disp('Starting simulation...');
simOut = sim(modelName);
disp('Simulation finished.');



