function q = IK_LBR_MED(p_e, R_e, elbow_sign)
%IK_LBR_MED  Closed-form inverse kinematics for KUKA LBR Med 7 R800
%
%  ONE closed-form solution via kinematic decoupling (class slides 45-53).
%  Redundancy resolution: q3 = 0  (upper-arm rotation fixed at zero).
%  This yields a determinate 6-DOF sub-problem solvable analytically.
%
%  INPUTS
%    p_e        [3x1]  desired end-effector position (metres)
%    R_e        [3x3]  desired end-effector rotation matrix
%    elbow_sign  +1 (elbow-down, default) or -1 (elbow-up)
%
%  OUTPUT
%    q          [7x1]  joint angles (rad): [q1 q2 q3 q4 q5 q6 q7]
%
%  KINEMATIC DECOUPLING PROCEDURE (Siciliano, slides 45-53):
%    Step 1 : Compute wrist center  p_W = p_e - d7*a_e
%    Step 2 : Solve q1 (base rotation)
%    Step 3 : Solve q4 (elbow) from sagittal-plane distance
%    Step 4 : Solve q2 (shoulder) from sagittal-plane geometry
%    Step 5 : Set q3 = 0 (redundancy resolution)
%    Step 6 : Compute R_W = R4^{0T} * R_e  (wrist subproblem)
%    Step 7 : ZYZ decomposition of R_W -> q5, q6, q7
%
%  SINGULARITY CONDITIONS:
%    (S1)  p_Wx = p_Wy = 0         : q1 indeterminate  (base singularity)
%    (S2)  q4 = 0 or pi            : boundary of workspace (elbow singularity)
%    (S3)  q2 = 0                  : arm aligned with z0 (shoulder singularity)
%    (S4)  q6 = 0 or pi            : z4 || z6 (wrist singularity)

if nargin < 3, elbow_sign = +1; end

d1 = 0.340;  d3 = 0.400;  d5 = 0.400;  d7 = 0.126;

%% STEP 1 — Wrist center
a_e  = R_e(:,3);               % approach vector (3rd column of R_e)
p_W  = p_e - d7 * a_e;        % wrist center = O5 = O6

%% STEP 2 — q1 (base rotation about z0)
% Warning S1: wrist on z0 axis -> q1 indeterminate
if norm(p_W(1:2)) < 1e-9
    warning('IK:BaseS','Singularity S1: wrist on z0, q1 set to 0');
    q1 = 0;
else
    q1 = atan2(p_W(2), p_W(1));
end

%% STEP 3 — q4 (elbow) from the sagittal-plane cosine law
L = norm(p_W(1:2));              % horizontal distance from z0
H = p_W(3) - d1;                % height above O1

D = (L^2 + H^2 - d3^2 - d5^2) / (2*d3*d5);

% Warning S2: boundary of workspace
if abs(D) > 1
    error('IK:Reach','Target unreachable (|D|=%.4f > 1)', abs(D));
end
if abs(abs(D)-1) < 1e-6
    warning('IK:ElbowS','Singularity S2: elbow at %s', ...
        ternary(D>0,'fully extended','folded'));
end

s4 = elbow_sign * sqrt(max(0, 1 - D^2));
q4 = atan2(s4, D);              % elbow angle

%% STEP 4 — q2 (shoulder tilt)
% Angle from vertical (z0) to the O2->O5 line in the sagittal plane
gamma = atan2(L, H);
% Angle in the triangle O2-O3-O5 at vertex O2
delta = atan2(d5*s4, d3 + d5*D);
q2 = gamma - delta;             % shoulder angle (elbow-down)

%% STEP 5 — q3 = 0  (redundancy resolution)
q3 = 0;

%% STEP 6 — Wrist rotation matrix R_W = R_4^{0T} * R_e
R4 = R4_from_q(q1, q2, q3, q4);
R_W = R4' * R_e;               % rotation from frame 4 to end-effector

%% STEP 7 — ZYZ-like decomposition for q5, q6, q7
% R_W = Rz(q5)*Ry'(q6)*Rz(q7)  [structure from DHTransf with alpha=pi/2,-pi/2,0]
% From the wrist DH chain, R7^4 has columns [n s a] where:
%   a = R_W(:,3) = [-c5*s6, -s5*s6, c6]
%   n_z = R_W(3,1) = s6*c7
%   s_z = R_W(3,2) = -s6*s7
r13 = R_W(1,3);  r23 = R_W(2,3);  r33 = R_W(3,3);
r31 = R_W(3,1);  r32 = R_W(3,2);

s6_sq = r13^2 + r23^2;

if s6_sq < 1e-10
    % Singularity S4: wrist singularity (q6=0 or pi)
    warning('IK:WristS','Singularity S4: wrist singularity, q6=0 or pi');
    q5 = 0;
    q6 = acos(min(1, max(-1, r33)));
    q7 = atan2(R_W(1,2), R_W(1,1));
else
    s6 = sqrt(s6_sq);
    q6 = atan2(s6, r33);       % q6 in (0,pi)
    q5 = atan2(-r23, -r13);    % from approach vector
    q7 = atan2(-r32,  r31);    % from bottom row
end

q = [q1; q2; q3; q4; q5; q6; q7];
end


%% ─── Helper: R of frame 4 expressed in frame 0 ─────────────────────────
function R4 = R4_from_q(q1, q2, q3, q4)
% Builds R4^0 by composing the rotation parts of A1..A4
c1=cos(q1); s1=sin(q1);
c2=cos(q2); s2=sin(q2);
c3=cos(q3); s3=sin(q3);
c4=cos(q4); s4=sin(q4);

% R2^0 (alpha1=pi/2, alpha2=-pi/2)
R2 = [ c1*c2, -s1, -c1*s2;
       s1*c2,  c1, -s1*s2;
       s2,     0,   c2   ];

% R3^0 = R2^0 * Rot(q3, alpha3=-pi/2)
R32 = [ c3,  0, -s3;
        s3,  0,  c3;
        0,  -1,   0];
R3 = R2 * R32;

% R4^0 = R3^0 * Rot(q4, alpha4=+pi/2)
R43 = [ c4,  0,  s4;
        s4,  0, -c4;
        0,   1,   0];
R4 = R3 * R43;
end


%% ─── Ternary helper ─────────────────────────────────────────────────────
function out = ternary(cond, a, b)
if cond, out=a; else, out=b; end
end
