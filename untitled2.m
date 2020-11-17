%% Using the Hopf fibration approach
clear; clc; close all;
set(0,'DefaultFigureWindowStyle','docked') %'normal' 'docked'
addpath(genpath('./utils'));

t=sym('t','real');
ax=sym('ax','real');
ay=sym('ay','real');
az=sym('az','real');
a=sym('a','real');
b=sym('b','real');
c=sym('c','real');
% psi=sym('psi','real');

T2=[t*t t 1]';
T3=[t*t*t t*t t 1]';
h=sym('h',[1,3],'real'); %coeff of psi
psi=h*T2;

P=sym('P%d%d',[3,4],'real');
Pt=P*T3;

g=sym('g','real'); %g is 9.81

qabc=qabcFromAccel([ax ay az],g);
qpsi=[cos(psi/2), 0, 0, sin(psi/2)]; %Note that qpsi has norm=1
q=multquat(qabc,qpsi); %Note that q is guaranteed to have norm=1
w_R_b=toRotMat(q);
w_fe=[1 1 1 1]';   %feature in world frame
w_T_b=[w_R_b Pt; zeros(1,3) 1];
k=invPose(w_T_b)*w_fe;
s=k(1:2)/k(3);
disp("simplifying...")
f=simplify(s'*s)
%%
close all; clc;clear;
import casadi.*

NoI=4; %number of intervals

jerk_cost=0;
psi_cost=0;
opti = casadi.Opti();

%Start and end of each interval
t0=0.0;
tf=1.0;

for n=1:NoI

P{n} = opti.variable(3,4);

V{n}=[0 3*P{n}(1,1) 2*P{n}(1,2) P{n}(1,3);
   0 3*P{n}(2,1) 2*P{n}(2,2) P{n}(2,3);
   0 3*P{n}(3,1) 2*P{n}(3,2) P{n}(3,3);
];

A{n}=[0    0     6*V{n}(1,2) 2*V{n}(1,3);
   0    0     6*V{n}(2,2) 2*V{n}(2,3);
   0    0     6*V{n}(3,2) 2*V{n}(3,3);
];

J{n}=[
   0    0    0   6*V{n}(1,2);
   0    0    0   6*V{n}(2,2);
   0    0    0   6*V{n}(3,2);
];

Psi{n} = [0 opti.variable(1,3)]; %0t^3 + at^2 + bt + c 

VPsi{n}=[0    0     2*Psi{n}(1,2) V{n}(1,3)];

APsi{n}=[0    0    0   2*Psi{n}(1,2);];

syms t
T=[t*t*t t*t t 1]';
T0=double(subs(T,t,t0));
Tf=double(subs(T,t,tf));

init_pos=[0;0;0];
init_vel=[0;0;0];
init_accel=[0;0;0];
final_pos=[2;0;0];
final_vel=[0;0;0];
final_accel=[0;0;0];


jerk=J{n}(:,end);
apsi=APsi{n}(:,end);

psi_cost=psi_cost+apsi*apsi; %apsi is a scalar
jerk_cost = jerk_cost + jerk'*jerk;


end

opti.minimize(  jerk_cost +0*psi_cost );

%Initial and final constraints
opti.subject_to( P{1}*T0==init_pos );
opti.subject_to( V{1}*T0==init_vel );
opti.subject_to( A{1}*T0==init_accel );
opti.subject_to( P{NoI}*Tf==final_pos );
opti.subject_to( V{NoI}*Tf==final_vel );
opti.subject_to( A{NoI}*Tf==final_accel );

opti.subject_to( Psi{1}*T0==0 );
% opti.subject_to( Psi{NoI}*Tf==2*pi );%Be careful with this
%     (wrapping psi)

%Continuity constraints
for n=1:(NoI-1)
    opti.subject_to( P{n+1}*T0==P{n}*Tf );
    opti.subject_to( V{n+1}*T0==V{n}*Tf );
    opti.subject_to( A{n+1}*T0==A{n}*Tf );

     opti.subject_to( Psi{n+1}*T0==Psi{n}*Tf ); %Be careful with this
%     (wrapping psi)
     opti.subject_to( VPsi{n+1}*T0==VPsi{n}*Tf );

end

opti.solver('ipopt');
sol = opti.solve();

for n=1:(NoI-1)
    P_guess{n}=sol.value(P{n});
end

g=9.81;
%Compute perception cost
perception_cost=0;

for n=1:(NoI-1)

    deltat=0.1;
    t_constrained=0.0:deltat:1.0;
    j=1;

    t=opti.variable(1,1);
    T=[t*t*t;t*t;t;1];
    At=A{n}*T;
    Pt=P{n}*T;
    axt=At(1,:); ayt=At(2,:); azt=At(3,:);
    psit=Psi{n}*T;

    qabc=qabcFromAccel([axt ayt azt],g);
    qpsi=[cos(psit/2), 0, 0, sin(psit/2)]; %Note that qpsi has norm=1
    q=multquat(qabc,qpsi); %Note that q is guaranteed to have norm=1
    w_R_b=toRotMat(q);
    w_fe=[1 1 1 1]';   %feature in world frame
    w_T_b=[w_R_b Pt; zeros(1,3) 1];

    b_T_c=[roty(90)*rotz(90) zeros(3,1); zeros(1,3) 1];

    k=invPose(w_T_b)*w_fe; %inv(b_T_c)*
    s=k(1:2)/k(3);
    f=s'*s; %I wanna minimize the integral of this funcion. Approx. using symp. Rule

    perception_cost_interval=0;
    for t_i=t_constrained
        perception_cost_interval=perception_cost_interval+getSimpsonCoeff(j,numel(t_constrained))*substitute(f,t,t_i);
        j=j+1;
    end

    perception_cost_interval=(deltat/3.0)*perception_cost_interval; %Only for completeness (Simpson's rule), doesn't affect optimization
    
   perception_cost=perception_cost+perception_cost_interval;
end

opti.minimize(0.005*jerk_cost + 0*psi_cost + perception_cost );

for n=1:(NoI-1)
    opti.set_initial(P{n}, P_guess{n})
end

jit_compilation=false;
opti.solver('ipopt',struct('jit',jit_compilation));
sol = opti.solve();


syms t real
T=[t*t*t t*t t 1]';
figure; hold on;

for n=1:NoI
    position=sol.value(P{n})*T;
    fplot3(position(1),position(2),position(3),[t0,tf]); 
end

plotSphere(init_pos,0.02,'r'); plotSphere(final_pos,0.02,'b'); plotSphere(w_fe(1:3),0.02,'g');
% plotSphere(double(subs(position,t,0.5)),0.02,'r');

view([45,45]); axis equal

disp("Plotting")
for n=1:NoI
    for ti=t0:0.03:tf  %t_constrained

        T=[ti^3 ti^2 ti 1]';

        w_t_b = sol.value(P{n})*T;
        accel = sol.value(A{n})*T;
        psiT=sol.value(Psi{n})*T;

        qabc=qabcFromAccel(accel, g);

        qpsi=[cos(psiT/2), 0, 0, sin(psiT/2)]; %Note that qpsi has norm=1
        q=multquat(qabc,qpsi); %Note that q is guaranteed to have norm=1

        w_R_b=toRotMat(q);
        w_T_b=[w_R_b w_t_b; 0 0 0 1];
        plotAxesArrowsT(0.2,w_T_b)

    end
    disp(['Done plotting interval ', num2str(n)])
end
%% 
%Using the Mellinger approach (Rot. matrix)
% for t=0.5;%0.1:0.5:1.0
% 
% ax=A(1,:)*[t*t*t;t*t;t;1];
% ay=A(2,:)*[t*t*t;t*t;t;1];
% az=A(3,:)*[t*t*t;t*t;t;1];
% 
% cost=cost + (P14 - (ay*(ay*cos(h1*t^2 + h2*t + h3) - ax*sin(h1*t^2 + h2*t + h3)) + cos(h1*t^2 + h2*t + h3)*(az + g)^2)/((az + g)^2*(ax*cos(h1*t^2 + h2*t + h3) + ay*sin(h1*t^2 + h2*t + h3))^2 + (ay*(ay*cos(h1*t^2 + h2*t + h3) - ax*sin(h1*t^2 + h2*t + h3)) + cos(h1*t^2 + h2*t + h3)*(az + g)^2)^2 + (ax*(ay*cos(h1*t^2 + h2*t + h3) - ax*sin(h1*t^2 + h2*t + h3)) - sin(h1*t^2 + h2*t + h3)*(az + g)^2)^2)^(1/2) + P13*t + P11*t^3 + P12*t^2)^2/(P34 + P33*t + P31*t^3 + P32*t^2 + ((az + g)*(ax*cos(h1*t^2 + h2*t + h3) + ay*sin(h1*t^2 + h2*t + h3)))/((az + g)^2*(ax*cos(h1*t^2 + h2*t + h3) + ay*sin(h1*t^2 + h2*t + h3))^2 + (ay*(ay*cos(h1*t^2 + h2*t + h3) - ax*sin(h1*t^2 + h2*t + h3)) + cos(h1*t^2 + h2*t + h3)*(az + g)^2)^2 + (ax*(ay*cos(h1*t^2 + h2*t + h3) - ax*sin(h1*t^2 + h2*t + h3)) - sin(h1*t^2 + h2*t + h3)*(az + g)^2)^2)^(1/2))^2 + (P24 + P23*t + (ax*(ay*cos(h1*t^2 + h2*t + h3) - ax*sin(h1*t^2 + h2*t + h3)) - sin(h1*t^2 + h2*t + h3)*(az + g)^2)/((az + g)^2*(ax*cos(h1*t^2 + h2*t + h3) + ay*sin(h1*t^2 + h2*t + h3))^2 + (ay*(ay*cos(h1*t^2 + h2*t + h3) - ax*sin(h1*t^2 + h2*t + h3)) + cos(h1*t^2 + h2*t + h3)*(az + g)^2)^2 + (ax*(ay*cos(h1*t^2 + h2*t + h3) - ax*sin(h1*t^2 + h2*t + h3)) - sin(h1*t^2 + h2*t + h3)*(az + g)^2)^2)^(1/2) + P21*t^3 + P22*t^2)^2/(P34 + P33*t + P31*t^3 + P32*t^2 + ((az + g)*(ax*cos(h1*t^2 + h2*t + h3) + ay*sin(h1*t^2 + h2*t + h3)))/((az + g)^2*(ax*cos(h1*t^2 + h2*t + h3) + ay*sin(h1*t^2 + h2*t + h3))^2 + (ay*(ay*cos(h1*t^2 + h2*t + h3) - ax*sin(h1*t^2 + h2*t + h3)) + cos(h1*t^2 + h2*t + h3)*(az + g)^2)^2 + (ax*(ay*cos(h1*t^2 + h2*t + h3) - ax*sin(h1*t^2 + h2*t + h3)) - sin(h1*t^2 + h2*t + h3)*(az + g)^2)^2)^(1/2))^2;
% 
% end

%Using the Hopf fibration approach

%Solve without perception cost (convex problem):
%%
close all; clc;
w_R_b=rotx(30);
w_t_b=[1 0 0]';
w_T_b=[w_R_b w_t_b; 0 0 0 1];
plotAxesArrowsT(0.2,w_T_b)


%% Using the Mellinger approach (Rot. matrix)
clc
syms t real
syms ax ay az real
syms g real
T2=[t*t t 1]';
T3=[t*t*t t*t t 1]';
h=sym('h',[1,3],'real'); %coeff of psi

P=sym('P%d%d',[3,4],'real');

Pt=P*T3;

psi=h*T2;

sp=sin(psi);
cp=cos(psi);

a=[ax ay az]';
zb=a+[0 0 g]';
xc=[cp sp 0]';
yb=cross(zb,xc);
xb=cross(yb,zb);
R=[xb/norm(xb) yb/norm(yb) zb/norm(zb)];


w_fe=[-1 0 0 1]';   %feature in world frame
k=[R Pt; zeros(1,3) 1]*w_fe;
s=k(1:2)/k(3);
f=simplify(s'*s)

% int(f,t,0,1)

%%
P=SX.sym('P',[3,4]);
t=SX.sym('t');
T=[t*t*t t*t t 1]';

PT=P*T;

VT=[gradient(PT(1),t);
    gradient(PT(2),t);
    gradient(PT(3),t)];

AT=[gradient(VT(1),t);
    gradient(VT(2),t);
    gradient(VT(3),t)];

JT=[gradient(AT(1),t);
    gradient(AT(2),t);
    gradient(AT(3),t)];

f=JT'*JT;

t0=0;
tf=1;

init_pos=[0 0 0]';
init_vel=[0 0 0]';

final_pos=[0 0 2]';

g=[ substitute(PT,t,t0)==init_pos;
%     substitute(VT,t,t0)==init_vel;
    substitute(PT,t,tf)==final_pos;
]

nlp = struct;            % NLP declaration
nlp.x = [P(:)];         % decision vars
nlp.f = f;               % objective
nlp.g = g;               % constraints



F = nlpsol('F','ipopt',nlp);

% Solve the problem using a guess
solution=F('x0',rand(size(P(:))),'ubg',0,'lbg',0)


x_opt = solution.x;
disp(x_opt)

P=full(reshape(x_opt,size(P)));

syms t real
T=[t*t*t t*t t 1]';
fplot(P*T,[t0 tf])



%%
sp=sin(psi)
cp=cos(psi)

a=[ax ay az]';

t=a+[0 0 g]';

zb=t
xc=[cp sp 0]';


yb=cross(zb,xc);

xb=cross(yb,zb);

R=[xb/norm(xb) yb/norm(yb) zb/norm(zb)]


%%
% Symbols/expressions
x = MX.sym('x');
y = MX.sym('y');
z = MX.sym('z');
f = x^2+100*z^2;
g = z+(1-x)^2-y;

nlp = struct;            % NLP declaration
nlp.x = [x;y;z];         % decision vars
nlp.f = f;               % objective
nlp.g = g;               % constraints

% Create solver instance
F = nlpsol('F','ipopt',nlp);

% Solve the problem using a guess
F('x0',[2.5 3.0 0.75],'ubg',0,'lbg',0)


%%
clc;
syms a b c real
syms q0 q1 q2 q3 real

eqs=[
    q0^2+q1^2-q2^2-q3^2==a
    2*(q0*q3+q1*q2)==b
    2*(q1*q3-q0*q2)==c
];

tmp=1/sqrt(2*(1+c));
q0=tmp*(1+c);
q1=tmp*(-b);
q2=tmp*a;
q3=0;

assume(a*a+b*b+c*c==1)

simplify(q0^2+q1^2-q2^2-q3^2)

%%

syms t p1 p2 p3 real
t=sym('t','real');
p0=sym('p0','real');
p1=sym('p1','real');
p2=sym('p2','real');
p3=sym('p3','real');


quatmultiply([0 1+p1 p2 p3],[cos(t),sin(t),0,0])'


% quaternion(t,t,t,t)
% *quaternion(cos(t),sin(t),0,0)

% solve(eqs, [q0 q1 q2 q3])


function result=getSimpsonCoeff(j,total)

    j_is_even = (rem(j, 2) == 0);

    %https://en.wikipedia.org/wiki/Simpson%27s_rule
    if(j==1 || j==total)  %Beginning or end
        result=1.0;
    elseif (j_is_even)
        result=4.0;
    else 
        result=2.0;
    end

end