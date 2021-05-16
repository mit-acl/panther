clear; clc; close all;

addpath(genpath('./../../submodules/minvo/src/utils'));
addpath(genpath('./../../submodules/minvo/src/solutions'));
addpath(genpath('./more_utils'));



   accel=sym('a%d', [3 1]);
   yaw=sym('y',[1 1]);
   
   syms c s real %note that cy is cos(yaw/2)

%     qpsi=[cos(yaw/2), 0, 0, sin(yaw/2)]; %Note that qpsi has norm=1
    
    qpsi=[c, 0, 0, s]; %Note that qpsi has norm=1
    
    qpsi=[1 0 0  0];
    
    g=0.0;
    
    
      %%%%% Option A
    qabc=qabcFromAccel(accel,g);
    q=multquat(qabc,qpsi); %Note that q is guaranteed to have norm=1
    w_R_b_uno=toRotMat(q);
        
    
    a=[accel(1); accel(2); accel(3)+g];
    na=sqrt(a(1)^2+a(2)^2+a(3)^2);
    syms na real
    

    q_tmp= [qpsi(1)*(na+a(3));
            -qpsi(1)*a(2)+qpsi(4)*a(1);
            qpsi(4)*a(2)+qpsi(1)*a(1);
            qpsi(4)*(na+a(3))];
    
    factor_tmp=(1/(2*na*(na+a(3))));
    R_tmp=toRotMat(q_tmp);
        
    w_R_b_dos=factor_tmp*R_tmp;
    %%
    
    assume(c^2+s^2==1)
    simplify(R_tmp)
    
    simplify(subs(w_R_b_dos, na, sqrt(a(1)^2+a(2)^2+a(3)^2))- w_R_b_uno)
    
    
%%%%%%% MELLINGER APPROACH
assumeAlso(s, 'real')
assumeAlso(c, 'real')
assumeAlso(c^2+s^2==1)
assumeAlso(a, 'real')
assumeAlso(na>=0)
zb=a/na;%[a b c]';
yb=cross(zb, [c s 0]'); yb=yb/norm(yb);
xb=cross(yb,zb);
R_Mellinger=simplify([xb yb zb]);

%%%%%%% Sertac Karaman APPROACH (paper "perception-aware time-optimal path parametrization for quadrotors"), see last line of the proof of Claim 1 of Section IV
% zb_nn=a;%[a b c]';   _nn stands for not normalized
% xb_nn=cross(cross([0;0;1],[c;s;0]),zb);
% yb_nn=cross(zb, xb); 
% R_Sertac_nn=simplify([xb_nn yb_nn zb_nn]);
% R_Sertac=simplify([xb_nn/norm(xb_nn) yb_nn/norm(yb_nn) zb_nn/norm(zb_nn)]);
zb=a/na;%[a b c]';
xb=cross(cross([0;0;1],[c;s;0]),zb);xb=xb/norm(xb);
yb=cross(zb, xb); yb=yb/norm(yb);
R_Sertac=simplify([xb yb zb]);

tmp=rand(); tmp_accel=rand(3,1);

R_Mellinger_value=double(subs(R_Mellinger,[a;na;c;s],[tmp_accel;norm(tmp_accel);cos(tmp);sin(tmp)]))
R_Sertac_value=double(subs(R_Sertac,[a;na;c;s],[tmp_accel;norm(tmp_accel);cos(tmp);sin(tmp)]))

% subs(R_Mellinger,c,cos(tmp));
% subs(R_Mellinger,s,sin(tmp));

%%
g=0.0;
a=rand(3,1);
yaw=rand();
qpsi=[cos(yaw/2), 0, 0, sin(yaw/2)]; %Note that qpsi has norm=1
qabc=qabcFromAccel(a,g);
q=multquat(qabc,qpsi); %Note that q is guaranteed to have norm=1
w_R_b_uno=toRotMat(q);
B=double(w_R_b_uno(:,1:2)*norm(a));
vpa(B*B' - a'*a*eye(3) + a*a')



[Q D]=eig(B*B') 


%%
P = sym('P', [3 3],'real')

Q=rand(3,3);

s=solve(P'*Q==zeros(3,3))

subs(P,s)

PtQ=P'*Q;


todos=PtQ(:)

M=equationsToMatrix(todos)

inv(M) 


%%
clc
opti = casadi.Opti();

x = opti.variable();
y = opti.variable();


opti.minimize((1-x)^2+(y-x^2)^2);

opti.solver('ipopt');
sol = opti.solve();

disp('==========================================')

plot(sol.value(x),sol.value(y),'o');


opti2=opti;
opti2.solve()

% a=opti.debug
