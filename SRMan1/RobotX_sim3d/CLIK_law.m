function q_dot = CLIK_law(q, p_d, R_d_flat)
Kp = 2*eye(3); Ko = 2*eye(3); k0 = 0.5;
q_mid = zeros(7,1);
dh = [0.340,0,pi/2; 0,0,-pi/2; 0.400,0,-pi/2; 0,0,pi/2; 0.400,0,pi/2; 0,0,-pi/2; 0.126,0,0];
T = eye(4); Tf = zeros(4,4,8); Tf(:,:,1) = T;
for i = 1:7
    d=dh(i,1); a=dh(i,2); al=dh(i,3);
    c=cos(q(i)); s=sin(q(i)); ca=cos(al); sa=sin(al);
    T = T*[c,-s*ca,s*sa,a*c; s,c*ca,-c*sa,a*s; 0,sa,ca,d; 0,0,0,1];
    Tf(:,:,i+1) = T;
end
R_e = T(1:3,1:3); p_e = T(1:3,4);
J = zeros(6,7);
for i = 1:7
    z = Tf(1:3,3,i); o = Tf(1:3,4,i);
    J(1:3,i) = cross(z,p_e-o); J(4:6,i) = z;
end
J_pinv = J' / (J*J' + 1e-6*eye(6));
e_p = p_d - p_e;
R_d = reshape(R_d_flat,3,3); Re = R_d*R_e';
th = acos(min(1,max(-1,(trace(Re)-1)/2)));
if abs(th) < 1e-8, e_O = zeros(3,1);
else
    r = [Re(3,2)-Re(2,3);Re(1,3)-Re(3,1);Re(2,1)-Re(1,2)]/(2*sin(th));
    e_O = r*th;
end
v = [Kp*e_p; Ko*e_O];
q_dot = J_pinv*v + (eye(7)-J_pinv*J)*(-k0*(q-q_mid));
end
