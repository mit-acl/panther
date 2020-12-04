
clc; clear; close all;
import casadi.*
opti = casadi.Opti();

x = opti.variable();
y = opti.variable();

initial_value=[4.5 8.7];

opti.minimize(x+2*y);
opti.subject_to(x*x>=2);
opti.subject_to(x^2+3*y^2==4);

opts.expand=true; %When this option is true, it goes WAY faster!

opti.solver('ipopt',opts);
opti.set_initial(x,initial_value(1))
opti.set_initial(y,initial_value(2))

sol = opti.solve();

disp("Solution using everything from MATLAB")
[sol.value(x) sol.value(y)]

%%Generate now the c code and create a shared library
solver = nlpsol('solver', 'ipopt', struct('x',opti.x,'f',opti.f,'g',opti.g,'p',opti.p));
solver.generate_dependencies('minimal_example.c');

system('clang -fPIC -shared -O3 minimal_example.c -o minimal_example.so'); %returns 0 if compilation is successful

opts.expand=false; %it's not supported when using already compiled code, see https://gitlab.syscop.de/-/snippets/3#LC98
solver = nlpsol('solver', 'ipopt', './minimal_example.so',opts);


solution = solver('x0',initial_value, 'lbg',full(evalf(opti.lbg)),'ubg',full(evalf(opti.ubg))); %full(evalf()) is needed because opti.lbg is NOT a double

disp("Solution using compiled code")
disp(solution.x)
