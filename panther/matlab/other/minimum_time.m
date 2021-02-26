clc; close all; clear;


%The notation of this file is based on the paper "Constrined time-optimal
%control of double integrator system and its application in MPC"
% https://iopscience.iop.org/article/10.1088/1742-6596/783/1/012024

global v_max a_max

v_max=5.0;
a_max=1.0;


all_po=-5:0.1:5.0;
all_v0=-5:0.1:5.0;
all_times=zeros(numel(all_po), numel(all_v0));

i=1; j=1;
for p0=all_po
    for v0=all_v0
        all_times(i,j)=getTotalTime(p0, v0);
        j=j+1;
    end
    i=i+1;
    j=1;
end

[X,Y] = meshgrid(all_po,all_v0);
contour(X,Y,all_times,500); hold on;
yline(-v_max, '--')
yline(v_max, '--'); ylim([-v_max-1, v_max+1])



function time=getTotalTime(p0, v0)

%The notation of this function is based on the paper "Constrined time-optimal
%control of double integrator system and its application in MPC"
% https://iopscience.iop.org/article/10.1088/1742-6596/783/1/012024


global v_max a_max

assert(v0<=v_max && v0>=-v_max)

pf=0.0;
vf=0.0;

x1=v0;
x2=p0;
x1r=vf;
x2r=pf;

k1=a_max; %Note that the paper uses u\in[-1,1]. But setting k1 to a_max has the same effect
k2=1.0; 

x1_bar=v_max;

B=(k2/(2*k1))*sign(-x1 + x1r)*(pow(x1,2)-pow(x1r,2)) + x2r;
C=(k2/(2*k1))*(pow(x1,2)+pow(x1r,2))  - (k2/k1)*pow(x1_bar,2)  + x2r;
D=(-k2/(2*k1))*(pow(x1,2)+pow(x1r,2))  + (k2/k1)*pow(x1_bar,2)  + x2r;

if((x2<=B) && (x2>=C))

    time=(-k2*(x1+x1r) + 2*sqrt(pow(k2,2)*pow(x1,2)  -k1*k2*( (k2/(2*k1))  * (pow(x1,2)-pow(x1r,2)) + x2 -x2r  )))/(k1*k2);

elseif((x2<=B) && (x2<C))
  
   time= (x1_bar-x1-x1r)/k1   + (pow(x1,2)+pow(x1r,2))/(2*k1*x1_bar) + (x2r-x2)/(k2*x1_bar);
    
elseif((x2>B) && (x2<=D))
    
    time=(k2*(x1+x1r) + 2*sqrt(pow(k2,2)*pow(x1,2)  + k1*k2*( (k2/(2*k1))  * (-pow(x1,2)+pow(x1r,2)) + x2 -x2r  )))/(k1*k2);
    
else %(x2>B) && (x2>D)
    
    time= (x1_bar+x1+x1r)/k1   + (pow(x1,2)+pow(x1r,2))/(2*k1*x1_bar) + (-x2r+x2)/(k2*x1_bar);
    
end

end


%Simply so that the code above is the same as the one in C++
function result=pow(a,b)
result=a^b;
end




% %% OLD CODE
% 
% p0=0.0;
% v0=-1.0;
% 
% pf=1.0;
% vf=-2.1;
% v_max=5.0;
% a_max=1.0;
% 
% syms t real
% syms t1 real
% syms t2 real
% syms T real
% 
% 
% % syms deltaT3 real
% 
% a_T1=sign(pf-p0)*a_max;
% a_T2=0.0;
% a_T3=sign(pf-p0)*(-a_max);
% v_T2=sign(pf-p0)*v_max;
% 
% 
% assert(v0<=v_max && -v_max<=v0)
% assert(vf<=v_max && -v_max<=vf)
% 
% %Last interval (t3 is still unknown)
% t0=0.0;
% deltaT1 = t1;
% deltaT2 = t2-t1;
% t2=(deltaT1 + deltaT2);
% t3=T %deltaT1 + deltaT2 + deltaT3;
% 
% deltaT3 = T-deltaT1-deltaT2;
% 
% p_t1=p0 + v0*t + 0.5*a_T1*t^2;
% p_t3=(0.5*a_T3)*t^2 + (vf-a_T3*t3)*t + (pf-(0.5*a_T3)*t3^2 - (vf-a_T3*t3)*t3);
% 
% v_t1=diff(p_t1,t);
% v_t3=diff(p_t3,t);
% 
% init_state=[subs(p_t1,t,t0), subs(diff(p_t1,t),t,t0), subs(diff(diff(p_t1,t),t),t,t0)]
% final_state=[subs(p_t3,t,t3), subs(diff(p_t3,t),t,t3), subs(diff(diff(p_t3,t),t),t,t3)]
% 
% 
% t1_tmp=double(solve(diff(p_t1,t)==v_T2,t))
% t2_tmp=solve(diff(p_t3,t)==v_T2,t);
% % deltaT3_tmp=(t3-t2_tmp)
% 
% p_at_t1=double(subs(p_t1, t, t1_tmp));
% 
% p_at_t2=double(simplify(subs(p_t3, t, t2_tmp)));
% 
% % if(p_at_t1==p_at_t2)
% %     disp("First case!")
% %     deltaT1_solution=t1_tmp;
% %     deltaT2_solution=0.0;
% %     deltaT3_solution= (t3-t2_tmp);
% %     solution = piecewise(t<t1, p_t1, t>=t1, p_t3);
%     
% if(p_at_t1<=p_at_t2)
% 
%     disp("Second case!") % a_max, 0.0, -a_max. v_max is reached
%     deltaT1_solution=t1_tmp;
%     deltaT2_solution=(p_at_t2-p_at_t1)/(sign(p_at_t2-p_at_t1)*v_max);
%     deltaT3_solution= (t3-t2_tmp);
%     T_solution=deltaT1_solution + deltaT2_solution +deltaT3_solution
%     
%     p_t1_solution=p_t1;
%     p_t2_solution = p_at_t1 + v_T2*(t-t1);
%     p_t3_solution = subs(p_t3, T, T_solution);
%     
% 
% 
% else
%     disp("Third case!") % a_max, 0.0, -a_max
%     t_tmp=solve(v_t1==v_t3, t)
%     
%     my_coeff=coeffs(p_t1-p_t3,t,'All');
%     
%     a=my_coeff(1); b=my_coeff(2); c=my_coeff(3);
%     T_solution=double(solve(b^2 - 4*a*c ==0, T));
%     T_solution=T_solution(T_solution>0);
%     
%     p_t3_solution=subs(p_t3, T, T_solution);
%     s=double(solve(p_t3_solution==p_t1,t));
%     deltaT1_solution=s(1); %The two solutions are the same one (double root)
%     
%     deltaT2_solution=0.0;
%     deltaT3_solution=(T_solution-deltaT1_solution-deltaT2_solution);
% 
%     p_t1_solution=p_t1; 
%     p_t2_solution=0.0; %Not used
%     
% 
%         
% end
% 
% t1_solution=double(deltaT1_solution);
% t2_solution=double(deltaT1_solution+deltaT2_solution);
% 
% 
% solution = piecewise(t<t1_solution, p_t1_solution, t1_solution<t<=t2_solution, p_t2_solution, t>t2_solution, p_t3_solution);
% 
% 
% vars=[deltaT1, deltaT2, deltaT3];
% vars_solution=[deltaT1_solution, deltaT2_solution, deltaT3_solution];
% 
% t3=double(deltaT1_solution + deltaT2_solution +deltaT3_solution);
% 
% solution=subs(solution, vars, vars_solution)
% 
% 
% subplot(3,1,1)
% fplot(solution,[0, t3]); hold on; title('pos')
%  xline(t1_solution,'--');xline(t2_solution,'--')
% 
% subplot(3,1,2)
% fplot(diff(solution,t),[0, t3]); hold on; title('vel')
% 
% subplot(3,1,3)
% fplot(diff(diff(solution,t),t),[0, t3]); hold on; title('accel')
% 
% [t1_solution, t2_solution, T_solution]
% 
% 
% 
% %%
% % deltaT1_solution=t1_solution;
% % deltaT2_solution= t2_solution-t1_solution;
% % 
% % vars=[deltaT1, deltaT2];
% % vars_solution=[deltaT1_solution, deltaT2_solution];
% % 
% % p_t1=subs(p_t1,vars, vars_solution)
% % % p_t2=subs(p_t2,vars, vars_solution)
% % p_t3=subs(p_t3,vars, vars_solution)
% % 
% % 
% % 
% % y = piecewise(t<t1, p_t1, t1<t<=t2, p_t2, t>t2, p_t3);
% % 
% % interv=[t0, t3];
% % subplot(3,1,1)
% % fplot (y,interv); title('pos'); xline(t1,'--');xline(t2,'--'); 
% % 
% % subplot(3,1,2)
% % fplot (diff(y,t),interv);  title('vel');xline(t1,'--');xline(t2,'--')
% % 
% % subplot(3,1,3)
% % fplot (diff(diff(y,t),t),interv);  title('accel');xline(t1,'--');xline(t2,'--')
% % 
% % 
% % % syms b c real
% % % % p_t3 = 0.5*a_T3*t_T3^2 + (a_max*deltaT3)*t_T3 +(pf - (a_max*deltaT3^2)/2);
% % % % s=solve([subs(p_t3,t,t3)==pf,...
% % % %        subs(diff(p_t3,t),t,t3)==vf],[b,c])
% % % 
% % % % t_T3=(t-t2);
% % % 
% % % t3=1.0
% % % a=0.5*a_T3
% % % b=vf-2*a*t3;
% % % c=pf-a*t3^2 -b*t3;
% % % 
% % % % p_t3=a*t_T3^2 + b*t_T3 + c;
% % %%
% % clc
% % syms t real
% % syms t3 real
% % 
% % p=(0.5*a_max)*t^2 + (vf-a_max*t3)*t + (pf-(0.5*a_max)*t3^2 - (vf-a_max*t3)*t3);
% % 
% % subs(p,t,t3)
% % 
% % subs(diff(p,t),t,t3)
% % 
% % subs(diff(diff(p,t),t),t,t3)
% % 
% % %%
% % 
% % deltaT1=abs((v_max-v0)/a_max);
% % deltaT3=abs((v_max-vf)/a_max);
% % 
% % 
% % a_T1=sign(pf-p0)*a_max;
% % a_T2=0.0;
% % a_T3=sign(pf-p0)*(-a_max);
% % v_T2=sign(pf-p0)*v_max;
% % 
% % p=p0+v0*deltaT1+0.5*a_T1*deltaT1^2;
% % q=pf-v_T2*deltaT3-0.5*a_T3*deltaT3^2;
% % 
% % deltaT2 = abs((q-p)/v_max);
% % 
% % syms t real
% % p_T1=p0 + v0*t + 0.5*a_T1*t^2;
% % 
% % t0=0;
% % t1=deltaT1;
% % t2=(deltaT1 + deltaT2);
% % t3=deltaT1 + deltaT2 + deltaT3;
% % 
% % 
% % t_T2=(t-t1);
% % p_T2=subs(p_T1,t,t1) + v_T2 * t_T2; %+ 0.5*a_T2*t_T2^2;
% % 
% % t_T3=(t-t2);
% % p_T3=subs(p_T2,t,t2) + v_T2 * t_T3 + 0.5*a_T3*t_T3^2;
% % 
% % 
% % y = piecewise(t<t1, p_T1, t1<t<=t2, p_T2, t>t2, p_T3);
% % 
% % interv=[t0, t3];
% % subplot(3,1,1)
% % fplot (y,interv); title('pos'); xline(t1,'--');xline(t2,'--'); hold on
% % 
% % fplot(q-(-v_T2*t_T3-0.5*a_T3*t_T3^2),[t2,t3])
% % 
% % subplot(3,1,2)
% % fplot (diff(y,t),interv);  title('vel');xline(t1,'--');xline(t2,'--')
% % 
% % subplot(3,1,3)
% % fplot (diff(diff(y,t),t),interv);  title('accel');xline(t1,'--');xline(t2,'--')
% % 
% % figure;
% % fplot(p_T1, [0,t3]); xline(t1,'--');xline(t2,'--'); hold on
% % fplot(q-(-v_T2*t_T3-0.5*a_T3*t_T3^2), [0,t3]); xline(t1,'--');xline(t2,'--'); hold on
% % fplot(v_max*t,[t1,t2])