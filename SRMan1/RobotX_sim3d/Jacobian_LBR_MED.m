function J = Jacobian_LBR_MED()
%Jacobian_LBR_MED  Symbolic 6x7 geometric Jacobian for KUKA LBR Med 7 R800
%
%  GEOMETRIC JACOBIAN DEFINITION (Siciliano, slide 54-60):
%  For each revolute joint i:
%    J_P_i = z_{i-1} x (p_e - O_{i-1})   [linear velocity contribution]
%    J_O_i = z_{i-1}                        [angular velocity contribution]
%
%  Full Jacobian:  J = [J_P; J_O]  (6 x 7)
%
%  where z_{i-1} = third column of R_{i-1}^0  (joint axis in base frame)
%        O_{i-1} = position of frame i-1 origin in base frame
%        p_e     = end-effector position (from full DK chain)
%
%  OUTPUT
%    J   [6x7] symbolic Jacobian, variables q1..q7

addpath(fullfile(fileparts(mfilename('fullpath')), '..', ...
    'Robotics_Symbolic_Matlab_Toolbox-2'));

Robot = LBR_MED();
n     = size(Robot, 1);   % 7 joints

% Full DK for end-effector position
T_full = DKin(Robot);
p_e    = T_full(1:3, 4);

J = sym(zeros(6, n));
T_i = sym(eye(4));          % accumulated transform up to frame i-1

for i = 1:n
    z_im1 = T_i(1:3, 3);   % z_{i-1} in base frame (3rd column)
    o_im1 = T_i(1:3, 4);   % O_{i-1} in base frame

    % Linear part: z_{i-1} cross (p_e - O_{i-1})
    J(1:3, i) = cross(z_im1, p_e - o_im1);
    % Angular part: z_{i-1}
    J(4:6, i) = z_im1;

    % Update accumulated transform
    T_i = T_i * DHTransf(Robot(i,:));
end

disp('Simplifying Jacobian (may take a while)...');
J = simplify(J);
end
