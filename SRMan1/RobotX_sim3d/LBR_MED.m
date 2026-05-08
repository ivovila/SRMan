function Robot = LBR_MED()
%LBR_MED  DH parameters for the KUKA LBR Med 7 R800 surgical manipulator
%
%   Standard Denavit-Hartenberg convention as taught in class (Siciliano).
%   Toolbox row format:  [d,  theta,  a,  alpha,  offset]
%   Class table format:  [a,  alpha,  d,  theta]  (columns reordered)
%
%   Frame assignment (algorithmic form, slides 38-39):
%     z_i  along joint i+1 axis
%     x_i  along common normal z_{i-1}->z_i, from joint i to joint i+1
%     y_i  completes right-handed frame
%
%   At home config (all q=0) the arm stands fully vertical:
%     z0,z2,z4,z6,z7  point upward  (+z0)
%     z1,z5           point along   (-y0)   [horizontal]
%     z3              points along  (+y0)   [horizontal]
%
%   Link dimensions (KUKA LBR Med 7 R800):
%     d1 = 0.340 m   base to shoulder  (joint 2 axis)
%     d3 = 0.400 m   shoulder to elbow (joint 4 axis)
%     d5 = 0.400 m   elbow to wrist    (joint 6 axis)
%     d7 = 0.126 m   wrist to flange
%     ai = 0         all consecutive axes intersect => no common-normal offset
%
%   DH table (class column order: a | alpha | d | theta):
%   Link |  a  |  alpha  |   d   | theta
%     1  |  0  |  +pi/2  | 0.340 |  q1
%     2  |  0  |  -pi/2  |   0   |  q2
%     3  |  0  |  -pi/2  | 0.400 |  q3
%     4  |  0  |  +pi/2  |   0   |  q4
%     5  |  0  |  +pi/2  | 0.400 |  q5
%     6  |  0  |  -pi/2  |   0   |  q6
%     7  |  0  |    0    | 0.126 |  q7

syms q1 q2 q3 q4 q5 q6 q7 real

%         d       theta   a    alpha    offset
Robot = [ 0.340,  q1,     0,   pi/2,   0;
          0,      q2,     0,  -pi/2,   0;
          0.400,  q3,     0,  -pi/2,   0;
          0,      q4,     0,   pi/2,   0;
          0.400,  q5,     0,   pi/2,   0;
          0,      q6,     0,  -pi/2,   0;
          0.126,  q7,     0,   0,      0];
end
