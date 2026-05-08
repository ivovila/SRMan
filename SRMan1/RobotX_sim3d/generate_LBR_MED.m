%% generate_LBR_MED.m
%  Generates the Simulink library LBR_MED_Lib.slx containing:
%    - LBR_MED_Direct_Kinematics  : embedded MATLAB function block
%                                   inputs  : q1..q7 (rad)
%                                   outputs : R (3x3), p (3x1, metres)
%    - Visual subsystem           : Sim3D 3D visualisation of DH frames
%
%  Run this script ONCE to generate the library, then open
%  LBR_MED_Simul.slx for validation (built by build_LBR_MED_Simul.m).
%
%  Requires: Symbolic Math Toolbox, Simulink, Sim3D (for 3-D visualisation).

addpath(fullfile(fileparts(mfilename('fullpath')), '..', ...
    'Robotics_Symbolic_Matlab_Toolbox-2'));

%% 1 - Symbolic direct kinematics
Robot = LBR_MED();
disp('Computing symbolic direct kinematics (may take ~30 s) ...');
T = DKin(Robot);
R = T(1:3, 1:3);
p = T(1:3, 4);
disp('Done.');

%% 2 - Create Simulink library
mdl = 'LBR_MED_Lib';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl, 'Library');
open_system(mdl);

%% 3 - Embedded MATLAB Function block for direct kinematics
matlabFunctionBlock([mdl '/LBR_MED_Direct_Kinematics'], R, p);

%% 4 - 3-D visualisation subsystem (Sim3D)
frameWrl = fullfile(fileparts(mfilename('fullpath')), 'frame_axes.wrl');
try
    buildDHActors(Robot, mdl, frameWrl);
catch ME
    warning('Sim3D visualisation skipped: %s', ME.message);
end

%% 5 - Save
save_system(mdl);
close_system(mdl);
fprintf('Library saved as %s.slx\n', mdl);
