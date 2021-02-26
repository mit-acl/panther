clc;
xi=10*rand(3,1);
phi=20*rand();

xi_bar=xi/norm(xi);
q_xi=(1/(sqrt(2*(1+xi_bar(3)))))*[1+xi_bar(3), -xi_bar(2) xi_bar(1) 0];

q_wb= multquat(q_xi,[cos(phi/2.0) 0 0 sin(phi/2.0)]');

norm(q_xi)

norm(q_wb)

toRotMat(q_wb)*[0;0;1] - xi_bar

toRotMat(q_xi)*[0;0;1] - xi_bar