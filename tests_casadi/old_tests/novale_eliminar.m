
clc; clear; close all;
opti = casadi.Opti();

x = opti.variable(); 
y = opti.variable();

opti.minimize( (y-x^2)^2   );
opti.subject_to( x^2+y^2==1 );
opti.subject_to(     x+y>=1 );

opti.solver('ipopt');

sol = opti.solve();

opti.set_initial(x,sol.value(x))
opti.set_initial(y,sol.value(y))

opts = struct;
opts.ipopt.warm_start_init_point = 'yes';
opts.ipopt.warm_start_bound_push=1e-9;
opts.ipopt.warm_start_bound_frac=1e-9;
opts.ipopt.warm_start_slack_bound_frac=1e-9;
opts.ipopt.warm_start_slack_bound_push=1e-9;
opts.ipopt.warm_start_mult_bound_push=1e-9;
opti.solver('ipopt',opts);

sol = opti.solve();