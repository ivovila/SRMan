%% validate_DK.m
%  Validates the KUKA LBR MED 7 R800 Direct Kinematics model.
%
%  Method: substitute four geometrically simple configurations into the
%  symbolic T matrix and compare with hand-computed expected results.
%
%  All four configs are constructed so the end-effector pose is
%  determinable by geometric reasoning alone (no numerics needed).

addpath(fullfile(fileparts(mfilename('fullpath')), '..', ...
    'Robotics_Symbolic_Matlab_Toolbox-2'));

syms q1 q2 q3 q4 q5 q6 q7 real

%% Compute symbolic DK once
Robot = LBR_MED();
disp('Computing symbolic direct kinematics ...');
T = DKin(Robot);
disp('Done.');

vars = {q1, q2, q3, q4, q5, q6, q7};

%--------------------------------------------------------------------------
% Helper: substitute a config and display
%--------------------------------------------------------------------------
function check(T, vars, qvals, label, p_exp, R_exp)
    T_num = double(subs(T, vars, num2cell(qvals)));
    R_num = T_num(1:3, 1:3);
    p_num = T_num(1:3, 4);

    fprintf('\n=== %s ===\n', label);
    fprintf('  q = [%s] rad\n', num2str(qvals, '%.4f '));
    fprintf('  p  (computed)  = [%6.4f  %6.4f  %6.4f] m\n', p_num);
    fprintf('  p  (expected)  = [%6.4f  %6.4f  %6.4f] m\n', p_exp);
    fprintf('  |p_err| = %.2e m\n', norm(p_num - p_exp(:)));
    fprintf('  R  (computed):\n'); disp(R_num);
    fprintf('  R  (expected):\n'); disp(R_exp);
    fprintf('  |R_err|_F = %.2e\n', norm(R_num - R_exp, 'fro'));
end

%--------------------------------------------------------------------------
% Config 1: q = 0  (arm fully extended upward)
%
%  Geometric reasoning:
%    All joints at 0 -> arm stands straight up.
%    Total height = d1+d3+d5+d7 = 0.340+0.400+0.400+0.126 = 1.266 m
%    p = [0, 0, 1.266]'   R = I3
%--------------------------------------------------------------------------
q1v = [0 0 0 0 0 0 0];
p1  = [0; 0; 1.266];
R1  = eye(3);
check(T, vars, q1v, 'Config 1: all q=0 (arm straight up)', p1, R1);

%--------------------------------------------------------------------------
% Config 2: q2 = pi/2, all others 0  (shoulder bent horizontally)
%
%  Geometric reasoning:
%    Joint 2 rotates about z1 (which at q1=0 points along -y0).
%    Rotating by +pi/2 tips the arm from vertical to the -x0 direction.
%    Upper arm (d3=0.4) + forearm (d5=0.4) + flange (d7=0.126) = 0.926 m
%    extend horizontally along -x0 from O1 = [0,0,0.340].
%    p = [-0.926, 0, 0.340]'
%    Rotation: approach vector a = -x0, slide s = y0, normal n = z0 × a
%--------------------------------------------------------------------------
q2v = [0 pi/2 0 0 0 0 0];
p2  = [-0.926; 0; 0.340];
R2  = [ 0  0 -1;
        0  1  0;
        1  0  0];
check(T, vars, q2v, 'Config 2: q2=pi/2 (shoulder horizontal, -x)', p2, R2);

%--------------------------------------------------------------------------
% Config 3: q1 = pi/2, all others 0  (base rotation, arm still up)
%
%  Geometric reasoning:
%    q1 is the base rotation about z0 (vertical).
%    The arm is straight (q2=...=q7=0) so rotating about z0 does NOT
%    move the end-effector position (it stays at [0,0,1.266]).
%    But the orientation of the end-effector rotates: R = Rz(pi/2).
%    p = [0, 0, 1.266]'   R = Rz(pi/2)
%--------------------------------------------------------------------------
q3v = [pi/2 0 0 0 0 0 0];
p3  = [0; 0; 1.266];
R3  = [0 -1  0;
       1  0  0;
       0  0  1];
check(T, vars, q3v, 'Config 3: q1=pi/2 (base spin, arm still up)', p3, R3);

%--------------------------------------------------------------------------
% Config 4: q2=pi/2, q4=pi/2, others 0  (elbow folded up)
%
%  Geometric reasoning:
%    q2=pi/2: upper arm (d3=0.4 m) extends along -x0 from [0,0,0.34].
%    q4=pi/2: forearm (d5=0.4 m) bends 90 deg UP from the elbow.
%    Elbow position: O3 = [-0.400, 0, 0.340]
%    Forearm goes vertically: adds [0, 0, 0.400]
%    Flange adds [0, 0, 0.126]
%    p = [-0.400, 0, 0.866]'   R = I3  (frames realign to base frame)
%--------------------------------------------------------------------------
q4v = [0 pi/2 0 pi/2 0 0 0];
p4  = [-0.400; 0; 0.866];
R4  = eye(3);
check(T, vars, q4v, 'Config 4: q2=pi/2, q4=pi/2 (elbow folded up)', p4, R4);

fprintf('\nAll configs checked. Errors < 1e-10 confirm correct DH table.\n');
