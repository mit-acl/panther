clc; clear; close all;


yaw_dot_max=1; %rad/s

n=20;
all_t=linspace(0,20,n);
all_phi=linspace(0,30,n);
all_phi=all_phi + yaw_dot_max*randm11(1,size(all_phi,2));

all_phi=wrapToPi(all_phi);
plot(all_t,all_phi, 'o' );

% all_phi_correct=all_phi(1);

all_phi_correct=all_phi(1);

for phi_i=all_phi(2:end)
    
    disp("----")
    
    previous_phi=all_phi_correct(end);    
    differ=previous_phi-phi_i;    
    phi_i_f=phi_i+floor(differ/(2*pi))*2*pi;
    phi_i_c=phi_i+ceil(differ/(2*pi))*2*pi; 
    
    if(abs((previous_phi-phi_i_f))<abs((previous_phi-phi_i_c)))
        phi_i_corrected=phi_i_f;
    else
        phi_i_corrected=phi_i_c;
    end
    
    fprintf("previous_phi = %f\n", rad2deg(previous_phi))
    fprintf("phi_i = %f\n", rad2deg(phi_i))
    fprintf("phi_i_f= %f\n",rad2deg(phi_i_f));
    fprintf("phi_i_c= %f\n",rad2deg(phi_i_c));
    fprintf("phi_i_corrected = %f\n", rad2deg(phi_i_corrected))
    
    all_phi_correct=[all_phi_correct phi_i_corrected];
end

hold on;
plot(all_t,all_phi_correct, 'o' )

assert(max(diff(all_phi_correct))<=pi)


%rand between -1 and 1
function r=randm11(s1,s2)
    r = -1 + (1+1)*rand(s1,s2);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 
% n=10;
% all_t=linspace(0,1,n);
% all_phi=2*pi*rand(1,n);
% 
% a=sdpvar(3,1);
% 
% cost=0;
% for i=1:numel(all_t)
%     x=a'*getT(all_t(i));
%     
%     z=intvar(1,1);
%     y=all_phi(i)+2*pi*z;
%     
%     size(x)
% 
%     cost=cost + (x-y)^2;
% end
% 
% 
% 
% settings=sdpsettings('usex0',1,'savesolveroutput',0,'savesolverinput',1,'solver','gurobi','showprogress',1,'verbose',2,'debug',1);
% 
% % settings=sdpsettings('showprogress',1,'verbose',2,'debug',1);
% % optimize([z>=-2*ones(size(z)),z<=2*ones(size(z)), cost>=0],cost,settings)
% 
% optimize([a>=zeros(size(a))],a'*a,settings)
% 
% function result=getT(ti)
%     result=[ti^2 ti 1]';
% end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 
% 
% close all; clc; clear;
% n=10;
% all_t=linspace(0,1,n);
% all_phi=2*pi*rand(1,n);
% 
% a=sym('a',[3,1],'real');
% 
% cost=0;
% for i=1:numel(all_t)
%     x=a'*getT(all_t(i));
%     y=all_phi(i);
%     tmp=getRot2d(x) - getRot2d(y);
%     %Note that frobNormSquared(tmp) = 4-4*cos(x-y), see https://www.wolframalpha.com/input/?i=%28cos%28x%29-cos%28y%29%29%5E2+%2B+%28-sin%28x%29%2Bsin%28y%29%29%5E2+%2B+%28sin%28x%29-sin%28y%29%29%5E2+%2B+%28cos%28x%29-cos%28y%29%29%5E2+
%     %cost=cost+frobNormSquared(tmp)
%     cost=cost - cos(x-y);
% end
% 
% fun=matlabFunction(cost);
% fun2 = @(x) fun(x(1),x(2),x(2));
% [a,fval] =fminunc(@(m) fun2(m),[1 1 1]);
% a=a';
% 
% % eq1=simplify(diff(cost,a(1)));
% % eq2=simplify(diff(cost,a(3)));
% % eq3=simplify(diff(cost,a(2)));
% % 
% % disp('solving the kkt equations')
% % s=vpasolve([eq1==0, eq2==0, eq3==0]);
% % s
% % a=subs(a,s);
% 
% figure; hold on;
% plot(all_t,all_phi,'o');
% 
% t=sym('t',[1,1]);
% fplot(a'*[t^2;t;1], [min(all_t),max(all_t)])
% 
% yline(0,'--');
% yline(2*pi,'--');
% 
% xlabel('t');
% ylabel('phi (rad)')
% 
% function result=getRot2d(x)
%   result=[cos(x) -sin(x); sin(x) cos(x)];
% end
% 
% function result=getT(ti)
%     result=[ti^2 ti 1]';
% end
% 
% % norm(XX,'fro')
% function result=frobNormSquared(A)
%     %TODO: assert A is 2x2
%    result=A(1,1)^2 + A(1,2)^2 + A(2,1)^2 + A(2,2)^2;
% end