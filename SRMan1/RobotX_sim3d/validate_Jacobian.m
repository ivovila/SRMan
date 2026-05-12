%% validate_Jacobian.m
%  Validates the geometric Jacobian via numerical differentiation of DK.
%  Also demonstrates rank drop at kinematic singularities.

addpath(fullfile(fileparts(mfilename('fullpath')), '..', ...
    'Robotics_Symbolic_Matlab_Toolbox-2'));

syms q1 q2 q3 q4 q5 q6 q7 real
Robot = LBR_MED();
T_sym = DKin(Robot);
J_sym = Jacobian_LBR_MED();
vars  = {q1,q2,q3,q4,q5,q6,q7};

DK_num = @(q) double(subs(T_sym, vars, num2cell(q(:)')));
J_num_sym = @(q) double(subs(J_sym,  vars, num2cell(q(:)')));

eps_fd = 1e-7;

fprintf('=== Jacobian Numerical Differentiation Validation ===\n\n');

configs = {
    [0, 0, 0, 0, 0, 0, 0],       'Home (all zeros)';
    [0, pi/4, 0, pi/4, 0, pi/4, 0], 'General config';
    [pi/3, pi/3, 0, pi/3, pi/4, pi/6, pi/5], 'Another config';
};

for k = 1:size(configs,1)
    q0 = configs{k,1};
    label = configs{k,2};

    % Symbolic Jacobian at q0
    J_s = J_num_sym(q0);

    % Numerical differentiation (finite differences, upper 3 rows = J_P)
    J_fd = zeros(6,7);
    for i = 1:7
        e = zeros(1,7); e(i) = eps_fd;
        Tp = DK_num(q0+e);  Tm = DK_num(q0-e);
        J_fd(1:3,i) = (Tp(1:3,4) - Tm(1:3,4)) / (2*eps_fd);
        % Angular part via skew: (dR/dq)*R' ≈ skew(omega)
        dR = (Tp(1:3,1:3) - Tm(1:3,1:3)) / (2*eps_fd);
        S  = dR * Tp(1:3,1:3)';          % approx skew(omega_i)
        J_fd(4:6,i) = [S(3,2); S(1,3); S(2,1)];
    end

    err = norm(J_s - J_fd, 'fro');
    fprintf('Config %d — %s\n', k, label);
    fprintf('  |J_sym - J_fd|_F = %.2e   (should be < 1e-4)\n', err);
    fprintf('  rank(J) = %d   (max=6)\n\n', rank(J_s));
end

%% Singularity demonstrations
fprintf('=== Singularity Analysis (rank of J) ===\n\n');

sing_configs = {
    [0, 0, 0, 0, 0, 0, 0],       'S2+S3: q2=0, q4=0 (arm straight, elbow at 0)';
    [0, 0, 0, pi, 0, 0, 0],      'S2: q4=pi (elbow fully extended)';
    [0, pi/4, 0, pi/4, 0, 0, 0], 'S4: q6=0 (wrist singularity)';
    [0, pi/4, 0, pi/4, 0, pi, 0],'S4: q6=pi (wrist singularity)';
};

for k = 1:size(sing_configs,1)
    q0    = sing_configs{k,1};
    label = sing_configs{k,2};
    J_s   = J_num_sym(q0);
    sv    = svd(J_s);
    r     = sum(sv > 1e-6);
    fprintf('  %s\n    rank=%d, smallest sv=%.2e\n\n', label, r, min(sv));
end
