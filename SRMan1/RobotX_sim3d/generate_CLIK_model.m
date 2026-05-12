%% generate_CLIK_model.m
%  Builds LBR_MED_CLIK.slx — Closed Loop Inverse Kinematics Simulink model.
%
%  CLIK LAW (Siciliano):
%    q_dot = J_pinv*[Kp*(p_d-p_e); Ko*e_O] + (I-J_pinv*J)*(-k0*(q-q_mid))
%
%  Run generate_LBR_MED.m first.

addpath(fullfile(fileparts(mfilename('fullpath')), '..', ...
    'Robotics_Symbolic_Matlab_Toolbox-2'));

scriptDir = fileparts(mfilename('fullpath'));
mdl = 'LBR_MED_CLIK';

if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

%% Gains
Kp = 2.0;  Ko = 2.0;  k0 = 0.5;

%% Write full CLIK implementation to CLIK_law.m (called by Simulink block)
lawLines = {
    sprintf('function q_dot = CLIK_law(q, p_d, R_d_flat)')
    sprintf('Kp = %g*eye(3); Ko = %g*eye(3); k0 = %g;', Kp, Ko, k0)
    'q_mid = zeros(7,1);'
    'dh = [0.340,0,pi/2; 0,0,-pi/2; 0.400,0,-pi/2; 0,0,pi/2; 0.400,0,pi/2; 0,0,-pi/2; 0.126,0,0];'
    'T = eye(4); Tf = zeros(4,4,8); Tf(:,:,1) = T;'
    'for i = 1:7'
    '    d=dh(i,1); a=dh(i,2); al=dh(i,3);'
    '    c=cos(q(i)); s=sin(q(i)); ca=cos(al); sa=sin(al);'
    '    T = T*[c,-s*ca,s*sa,a*c; s,c*ca,-c*sa,a*s; 0,sa,ca,d; 0,0,0,1];'
    '    Tf(:,:,i+1) = T;'
    'end'
    'R_e = T(1:3,1:3); p_e = T(1:3,4);'
    'J = zeros(6,7);'
    'for i = 1:7'
    '    z = Tf(1:3,3,i); o = Tf(1:3,4,i);'
    '    J(1:3,i) = cross(z,p_e-o); J(4:6,i) = z;'
    'end'
    'J_pinv = J'' / (J*J'' + 1e-6*eye(6));'
    'e_p = p_d - p_e;'
    'R_d = reshape(R_d_flat,3,3); Re = R_d*R_e'';'
    'th = acos(min(1,max(-1,(trace(Re)-1)/2)));'
    'if abs(th) < 1e-8, e_O = zeros(3,1);'
    'else'
    '    r = [Re(3,2)-Re(2,3);Re(1,3)-Re(3,1);Re(2,1)-Re(1,2)]/(2*sin(th));'
    '    e_O = r*th;'
    'end'
    'v = [Kp*e_p; Ko*e_O];'
    'q_dot = J_pinv*v + (eye(7)-J_pinv*J)*(-k0*(q-q_mid));'
    'end'
};
fid = fopen(fullfile(scriptDir, 'CLIK_law.m'), 'w');
fprintf(fid, '%s\n', strjoin(lawLines, newline));
fclose(fid);

%% Tiny wrapper injected into the Simulink MATLAB Function block
wrapSrc = sprintf('function q_dot = CLIK(q, p_d, R_d_flat)\nq_dot = CLIK_law(q, p_d, R_d_flat);\nend\n');

%% Add blocks (no VectorParams1D — not valid in modern MATLAB)
add_block('simulink/Sources/Constant', [mdl '/p_d'], ...
    'Value', '[0;0;1.266]', 'OutDataTypeStr', 'double', ...
    'Position', [50,50,160,80]);

add_block('simulink/Sources/Constant', [mdl '/R_d'], ...
    'Value', '[1 0 0 0 1 0 0 0 1]', 'OutDataTypeStr', 'double', ...
    'Position', [50,120,160,150]);

add_block('simulink/Continuous/Integrator', [mdl '/Integrator'], ...
    'InitialCondition', '[0.1;0.1;0;0.1;0;0.1;0]', ...
    'Position', [500,100,560,150]);

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/CLIK'], 'Position', [280,50,460,210]);

add_block('simulink/Sinks/To Workspace', [mdl '/q_out'], ...
    'VariableName', 'q_CLIK', 'SaveFormat', 'Array', ...
    'Position', [630,100,730,140]);

add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [630,40,690,80]);

%% Inject wrapper into MATLAB Function block via Stateflow API
rt = sfroot;
m  = rt.find('-isa','Stateflow.EMChart','Path',[mdl '/CLIK']);
if isempty(m)
    m = rt.find('-isa','Stateflow.EMChart','Name','CLIK');
end
if isempty(m)
    m = rt.find('-isa','Stateflow.EMFunction','Name','CLIK');
end
if ~isempty(m)
    m(1).Script = wrapSrc;
    disp('CLIK wrapper injected into block.');
else
    warning('Stateflow injection failed — paste contents of CLIK_law.m into the CLIK block manually.');
end

%% Save to script directory, close, reopen — forces port creation from function signature
slxPath = fullfile(scriptDir, [mdl '.slx']);
save_system(mdl, slxPath);
close_system(mdl, 0);
load_system(slxPath);
open_system(mdl);

%% Add connections (ports now exist after reopen)
add_line(mdl, 'Integrator/1', 'CLIK/1',       'autorouting','on');
add_line(mdl, 'p_d/1',        'CLIK/2',       'autorouting','on');
add_line(mdl, 'R_d/1',        'CLIK/3',       'autorouting','on');
add_line(mdl, 'CLIK/1',       'Integrator/1', 'autorouting','on');
add_line(mdl, 'Integrator/1', 'q_out/1',      'autorouting','on');
add_line(mdl, 'CLIK/1',       'Scope/1',      'autorouting','on');

%% Simulation settings
set_param(mdl, 'StopTime', '10', 'Solver', 'ode45');
save_system(mdl, slxPath);

fprintf('CLIK model saved: %s\n', slxPath);
fprintf('Press Ctrl+T in Simulink to run. Then run validate_CLIK.m for plots.\n');
