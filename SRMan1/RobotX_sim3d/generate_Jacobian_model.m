%% generate_Jacobian_model.m
%  Adds a Jacobian MATLAB Function block to LBR_MED_Lib.slx.
%  Inputs: q (7x1).  Output: J (6x7).
%  Run AFTER generate_LBR_MED.m.

addpath(fullfile(fileparts(mfilename('fullpath')), '..', ...
    'Robotics_Symbolic_Matlab_Toolbox-2'));

mdl = 'LBR_MED_Lib';
if ~bdIsLoaded(mdl), load_system(mdl); end
set_param(mdl, 'Lock', 'off');

%% Compute symbolic Jacobian and generate block
disp('Computing symbolic Jacobian...');
J = Jacobian_LBR_MED();
disp('Done.');

matlabFunctionBlock([mdl '/LBR_MED_Jacobian'], J);

save_system(mdl);
fprintf('Jacobian block added to %s.slx\n', mdl);
