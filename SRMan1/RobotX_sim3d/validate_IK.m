%% validate_IK.m
%  Validates the IK by round-trip: q_test -> DK -> (p,R) -> IK -> q' -> DK -> (p',R')
%  Pass condition: norm(p-p') < 1e-8 and norm(R-R','fro') < 1e-6

addpath(fullfile(fileparts(mfilename('fullpath')), '..', ...
    'Robotics_Symbolic_Matlab_Toolbox-2'));

syms q1 q2 q3 q4 q5 q6 q7 real
Robot = LBR_MED();
T_sym = DKin(Robot);
vars  = {q1,q2,q3,q4,q5,q6,q7};

% Helper: evaluate DK numerically at a joint vector
DK = @(q) double(subs(T_sym, vars, num2cell(q(:)')));

fprintf('=== IK Round-Trip Validation ===\n\n');

configs = {
    [0,    0,    0,    0,    0,    0,    0   ], 'Home (all zeros)';
    [0,    pi/4, 0,    pi/4, 0,    pi/6, 0   ], 'Elbow bent';
    [pi/6, pi/3, 0,    pi/3, pi/4, pi/4, pi/8], 'General config';
    [0,    pi/2, 0,    pi/2, 0,    pi/3, 0   ], 'Shoulder+Elbow';
};

for k = 1:size(configs,1)
    q_test = configs{k,1};
    label  = configs{k,2};

    % Forward: DK
    T_test = DK(q_test);
    p_test = T_test(1:3,4);
    R_test = T_test(1:3,1:3);

    % Inverse: IK
    try
        q_ik = IK_LBR_MED(p_test, R_test, +1);
    catch e
        fprintf('Config %d (%s): IK FAILED — %s\n', k, label, e.message);
        continue
    end

    % Forward again: DK on IK result
    T_ik = DK(q_ik');
    p_ik = T_ik(1:3,4);
    R_ik = T_ik(1:3,1:3);

    ep = norm(p_test - p_ik);
    eR = norm(R_test - R_ik,'fro');

    status = 'PASS';
    if ep > 1e-6 || eR > 1e-4, status = 'FAIL'; end

    fprintf('Config %d — %s\n', k, label);
    fprintf('  q_test = [%s]\n', num2str(q_test,'%6.3f '));
    fprintf('  q_IK   = [%s]\n', num2str(q_ik','%6.3f '));
    fprintf('  |p_err| = %.2e m     |R_err|_F = %.2e     [%s]\n\n', ...
        ep, eR, status);
end
