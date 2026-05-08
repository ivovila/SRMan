%% generate_IK_model.m
%  Adds an IK MATLAB Function block to LBR_MED_Lib.slx.
%  Run AFTER generate_LBR_MED.m.

addpath(fullfile(fileparts(mfilename('fullpath')), '..', ...
    'Robotics_Symbolic_Matlab_Toolbox-2'));

mdl = 'LBR_MED_Lib';
if ~bdIsLoaded(mdl), load_system(mdl); end

%% Add IK block as an embedded MATLAB Function
% The block wraps IK_LBR_MED.m; inputs: p_e(3), R_e(9=flat), elbow_sign(1)
blkName = [mdl '/LBR_MED_IK'];
if getSimulinkBlockHandle(blkName) ~= -1
    delete_block(blkName);
end
add_block('simulink/User-Defined Functions/MATLAB Function', blkName, ...
    'Position', [400 200 600 300]);

% Inject source code into the MATLAB Function block
rt = sfroot;
mf = rt.find('-isa','Stateflow.EMFunction','Name','LBR_MED_IK');
if ~isempty(mf)
    mf(1).Script = fileread('IK_LBR_MED.m');
end

save_system(mdl);
fprintf('IK block added to %s.slx\n', mdl);
