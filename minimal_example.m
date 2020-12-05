clc; clear; close all;
import casadi.*
opti = casadi.Opti();

x = opti.variable(1,1);
y = opti.variable(1,1);
p1 = opti.parameter(2,1);
p2 = opti.parameter(1,1);

initial_value=[4.5 8.7];
value_p1=[1.0;2.0];
value_p2=4.0;


opti.minimize(p1(1)*x+p2*p1(2)*y);
opti.subject_to(x*x>=2.5);
opti.subject_to(x^2+3*y^2==4.1);

opts.expand=true; %When this option is true, it goes WAY faster!

opti.solver('ipopt',opts);
opti.set_initial(x,initial_value(1))
opti.set_initial(y,initial_value(2))
opti.set_value(p1,value_p1); %Set a value to the parameter
opti.set_value(p2,value_p2); %Set a value to the parameter
sol = opti.solve();

disp("Solution using everything from MATLAB")
[sol.value(x) sol.value(y)]

%%Generate now the c code and create a shared library
solver1 = nlpsol('solver', 'ipopt', struct('x',opti.x,'f',opti.f,'g',opti.g,'p',opti.p));
solver1.generate_dependencies('minimal_example.c');

system('clang -fPIC -shared -O3 minimal_example.c -o minimal_example.so'); %returns 0 if compilation is successful

opts.expand=false; %it's not supported when using already compiled code, see https://gitlab.syscop.de/-/snippets/3#LC98
solver2 = nlpsol('solver', 'ipopt', './minimal_example.so',opts);

%Available names are:  [lbg, ubg, lbx, ubx, x0, lam_x0, lam_g0]. The first two ones are available doing opti.XXXX
solution = solver2('x0',initial_value,'p',[value_p1;value_p2],'lbg',full(evalf(opti.lbg)),'ubg',full(evalf(opti.ubg))); %full(evalf()) is needed because opti.lbg is NOT a double

disp("Solution using compiled code")
disp(solution.x)


%%
clc; clear; close all;

opti = casadi.Opti();
x = opti.variable(2,1);
y = opti.variable();
p = opti.parameter();
      
opti.minimize(y.^2+sin(x(1,1)-y-p).^2)
opti.subject_to(x(1,1)+y>=1)
opti.subject_to(x(1,1)+y<=100)
opti.subject_to(x(2,1)==0)
      
opti.solver('ipopt')
% opti.solver('sqpmethod',struct('qpsol','qrqp'));

%See example https://web.casadi.org/blog/nlp_sens/
% and https://github.com/casadi/casadi/issues/2499
% and https://lirias.kuleuven.be/retrieve/542250
pv=[x;y]; %primal variables

my_function = opti.to_function('my_function',{p,pv},{pv}); %opti.lam_g
      
initial_guess_pv=[0.1,0.2, 0];
my_function(3,initial_guess_pv) %Solve the problem for this given parameter and initial guess

%In case you wanna generate c code
my_function.generate('other.c') %THIS DOESN'T WORK IF YOU ARE USING ipopt AS THE SOLVER, SEE
                      % https://github.com/casadi/casadi/issues/2682  and
                      % https://groups.google.com/g/casadi-users/c/fgzohGQj-h8/m/8pXm6YovEwAJ#c34:~:text=This%20must%20be%20combined%20with%20a,self%2Dcontained%20C%20code%20at%20the%20moment

my_function.save('my_function.casadi')% See v3.5.5 Release notes https://github.com/casadi/casadi/releases

% f = Function('f',{}, {opti.lbg, opti.ubg, });
% f.generate('other.c')