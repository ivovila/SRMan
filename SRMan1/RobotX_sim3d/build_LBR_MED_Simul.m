%% build_LBR_MED_Simul.m
%  Builds the Simulink simulation model LBR_MED_Simul.slx for direct
%  kinematics validation.
%
%  Block layout:
%    [Constant q1..q7] --> [Mux 7x1] --> [DK block] --> [R out]
%                                                     --> [p out]
%
%  Change the Constant block values to switch between validation configs.
%  Run generate_LBR_MED.m first to create LBR_MED_Lib.slx.

addpath(fullfile(fileparts(mfilename('fullpath')), '..', ...
    'Robotics_Symbolic_Matlab_Toolbox-2'));

mdl     = 'LBR_MED_Simul';
libName = 'LBR_MED_Lib';

if ~exist([libName '.slx'], 'file')
    error('Run generate_LBR_MED.m first to create %s.slx', libName);
end

if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

load_system(libName);

%% --- Layout constants --------------------------------------------------
xConst = 50;   yConst = 50;  dy = 60;   h = 40;  w = 80;
xMux   = 220;  yMux   = 230;
xDK    = 360;  yDK    = 200;
xDispR = 560;  yDispR = 150;
xDispp = 560;  yDispp = 280;

%% --- 7 Constant blocks (joint angles) ----------------------------------
% Validation Config 1: q = zeros  (arm fully extended upward)
% Change these values to test other configurations.
q_vals = [0; 0; 0; 0; 0; 0; 0];   % rad — edit for each validation config

for i = 1:7
    blk = [mdl sprintf('/q%d', i)];
    add_block('simulink/Sources/Constant', blk, ...
        'Value',    num2str(q_vals(i)), ...
        'Position', [xConst, yConst+(i-1)*dy, xConst+w, yConst+(i-1)*dy+h]);
end

%% --- DK function block (from library) ----------------------------------
dkBlk = [mdl '/DK'];
add_block([libName '/LBR_MED_Direct_Kinematics'], dkBlk, ...
    'Position', [xDK, yDK, xDK+160, yDK+100]);

for i = 1:7
    add_line(mdl, sprintf('q%d/1', i), sprintf('DK/%d', i), 'autorouting', 'on');
end

%% --- Display blocks for R and p ----------------------------------------
dispR = [mdl '/R (3x3)'];
add_block('simulink/Sinks/Display', dispR, ...
    'Position', [xDispR, yDispR, xDispR+120, yDispR+60], ...
    'Format', 'short');
add_line(mdl, 'DK/1', 'R (3x3)/1', 'autorouting', 'on');

dispp = [mdl '/p (m)'];
add_block('simulink/Sinks/Display', dispp, ...
    'Position', [xDispp, yDispp, xDispp+120, yDispp+60], ...
    'Format', 'short');
add_line(mdl, 'DK/2', 'p (m)/1', 'autorouting', 'on');

%% --- To Workspace (for scripted comparison) ----------------------------
wsR = [mdl '/R_ws'];
add_block('simulink/Sinks/To Workspace', wsR, ...
    'VariableName', 'R_out', ...
    'Position', [xDispR, yDispR+80, xDispR+120, yDispR+120]);
add_line(mdl, 'DK/1', 'R_ws/1', 'autorouting', 'on');

wsp = [mdl '/p_ws'];
add_block('simulink/Sinks/To Workspace', wsp, ...
    'VariableName', 'p_out', ...
    'Position', [xDispp, yDispp+80, xDispp+120, yDispp+120]);
add_line(mdl, 'DK/2', 'p_ws/1', 'autorouting', 'on');

%% --- Save --------------------------------------------------------------
save_system(mdl);
fprintf('Simulation model saved as %s.slx\n', mdl);
fprintf('Open it in Simulink, set joint angles, and run (Ctrl+T).\n');
