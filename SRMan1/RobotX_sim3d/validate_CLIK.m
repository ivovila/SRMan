%% validate_CLIK.m
%  Validates CLIK convergence to two target poses.
%  Targets are computed from FK of non-singular reference configs —
%  this guarantees reachability and correct IK round-trip comparison.

addpath(fullfile(fileparts(mfilename('fullpath')), '..', ...
    'Robotics_Symbolic_Matlab_Toolbox-2'));

%% DH parameters [d, a, alpha] per row — KUKA LBR MED 7 R800
dh = [0.340,0, pi/2;
      0,    0,-pi/2;
      0.400,0,-pi/2;
      0,    0, pi/2;
      0.400,0, pi/2;
      0,    0,-pi/2;
      0.126,0, 0   ];

%% CLIK gains
Kp = 2*eye(3);  Ko = 2*eye(3);  k0 = 0.5;
q_mid = zeros(7,1);

%% Integration settings
dt = 0.01;  T = 10;  N = round(T/dt);
t_log = (0:N-1)*dt;

%% Compute target poses from FK — avoids singularities by construction
% Config A: general non-singular arm pose
q_ref_A = [pi/6; pi/3; 0; pi/3; 0; pi/4; 0];
% Config B: elbow folded (non-zero q6 avoids wrist singularity)
q_ref_B = [0; pi/2; 0; pi/2; 0; pi/3; 0];

[R_d_A, p_d_A, ~] = fk_jac(q_ref_A, dh);
[R_d_B, p_d_B, ~] = fk_jac(q_ref_B, dh);

fprintf('Target poses (from FK):\n');
fprintf('  Config A: p = [%.4f  %.4f  %.4f]\n', p_d_A);
fprintf('  Config B: p = [%.4f  %.4f  %.4f]\n\n', p_d_B);

targets = {p_d_A, R_d_A, 'Config A: general arm pose';
           p_d_B, R_d_B, 'Config B: elbow folded'    };

figure('Name','CLIK Convergence','Position',[100 100 900 600]);

for tc = 1:2
    p_d   = targets{tc,1};
    R_d   = targets{tc,2};
    label = targets{tc,3};

    q      = [0.1; 0.2; 0; 0.1; 0; 0.1; 0];   % initial config (near home)
    ep_log = zeros(1,N);
    eO_log = zeros(1,N);

    for k = 1:N
        [R_e, p_e, J] = fk_jac(q, dh);

        e_p = p_d - p_e;
        Re  = R_d * R_e';
        th  = acos(min(1,max(-1,(trace(Re)-1)/2)));
        if abs(th) < 1e-8
            e_O = zeros(3,1);
        else
            r   = [Re(3,2)-Re(2,3); Re(1,3)-Re(3,1); Re(2,1)-Re(1,2)] / (2*sin(th));
            e_O = r * th;
        end

        ep_log(k) = norm(e_p);
        eO_log(k) = norm(e_O);

        J_pinv = J' / (J*J' + 1e-6*eye(6));
        v      = [Kp*e_p; Ko*e_O];
        q_dot  = J_pinv*v + (eye(7)-J_pinv*J)*(-k0*(q-q_mid));
        q      = q + dt*q_dot;
    end
    q_CLIK = q;

    %% Plots
    subplot(2,2,(tc-1)*2+1);
    semilogy(t_log, ep_log, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('|e_p| (m)'); grid on;
    title(sprintf('%s — Position error', label));

    subplot(2,2,(tc-1)*2+2);
    semilogy(t_log, eO_log, 'r', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('|e_O| (rad)'); grid on;
    title(sprintf('%s — Orientation error', label));

    %% IK comparison
    q_IK = IK_LBR_MED(p_d, R_d, +1);
    [~, p_IK_fwd] = fk_jac(q_IK(:), dh);
    [~, p_CL_fwd] = fk_jac(q_CLIK,  dh);

    fprintf('=== %s ===\n', label);
    fprintf('  Joint |  q_IK (rad)  | q_CLIK (rad)\n');
    fprintf('  ------+--------------+--------------\n');
    for i = 1:7
        fprintf('    q%d  |  %9.4f   |  %9.4f\n', i, q_IK(i), q_CLIK(i));
    end
    fprintf('  Final errors: |e_p| = %.2e m,  |e_O| = %.2e rad\n', ep_log(end), eO_log(end));
    fprintf('  p_e (IK  ): [%.4f  %.4f  %.4f]\n', p_IK_fwd);
    fprintf('  p_e (CLIK): [%.4f  %.4f  %.4f]\n', p_CL_fwd);
    fprintf('  End-effector agreement |Dp| = %.2e m\n\n', norm(p_IK_fwd - p_CL_fwd));
end

%% Helper: numeric FK + geometric Jacobian
function [R_e, p_e, J] = fk_jac(q, dh)
    T = eye(4); Tf = zeros(4,4,8); Tf(:,:,1) = T;
    for i = 1:7
        d=dh(i,1); a=dh(i,2); al=dh(i,3);
        c=cos(q(i)); s=sin(q(i)); ca=cos(al); sa=sin(al);
        T = T * [c,-s*ca,s*sa,a*c; s,c*ca,-c*sa,a*s; 0,sa,ca,d; 0,0,0,1];
        Tf(:,:,i+1) = T;
    end
    R_e = T(1:3,1:3); p_e = T(1:3,4);
    J = zeros(6,7);
    for i = 1:7
        z = Tf(1:3,3,i); o = Tf(1:3,4,i);
        J(1:3,i) = cross(z,p_e-o); J(4:6,i) = z;
    end
end
