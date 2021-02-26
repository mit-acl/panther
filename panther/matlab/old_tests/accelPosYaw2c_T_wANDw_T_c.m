
function [c_Tcasadi_w, w_Tcasadi_c]=accelPosYaw2c_T_wANDw_T_c(sp,sy,j, u_casadi)

syms ax ay az real
syms yaw real
syms px py pz real
syms g real %g should be 9.81
syms u real

w_t_b = sp.evalDerivativeU(0,u,j); % sp.getPosU(u,j)
accel = sp.getAccelU(u,j);
yaw= sy.getPosU(u,j);


b_T_c=[roty(90)*rotz(-90) zeros(3,1); zeros(1,3) 1];


qabc=qabcFromAccel(accel,g);
qpsi=[cos(yaw/2), 0, 0, sin(yaw/2)]; %Note that qpsi has norm=1
q=simplify(multquat(qabc,qpsi)); %Note that q is guaranteed to have norm=1
w_R_b=simplify(toRotMat(q));

w_T_b=simplify([w_R_b w_t_b; zeros(1,3) 1]);


w_T_c=simplify(w_T_b*b_T_c);
c_T_b=invPose(b_T_c);
b_T_w=invPose(w_T_b);
c_T_w=simplify(c_T_b*b_T_w);

c_Tcasadi_w=sym2Casadi(c_T_w, {accel, pos, yaw, u}, {accel_casadi, pos_casadi, yaw_casadi, u_casadi});
w_Tcasadi_c=toC(w_T_c);

end
% w_fe=[1 1 1 1]';   %feature in world frame
% c_P=c_T_w*w_fe; %Position of the feature in the camera frame
% 
% 
% c_P=c_T_w*w_fe; %Position of the feature in the camera frame
% s=c_P(1:2)/(c_P(3));  
%   
%  s_dot=jacobian(s,u);