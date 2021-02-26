%Jesus Tordesillas, jtorde@mit.edu, December 2020

% Example of how to run this function:
% clc; clear;
% opti = casadi.Opti();
% import casadi.*
% x_sym=sym('x_%d_%d',[2,2]);
% x_casadi = opti.variable(2,2);
% u_sym=sym('u_%d_%d',[2,2]);
% u_casadi=opti.variable(2,2);
% 
% exp_sym = [3 4]*(x_sym+u_sym)*[3;4];    
% 
% exp_casadi=sym2casadi(exp_sym, {x_sym, u_sym}, {x_casadi, u_casadi})


function exp_casadi=sym2casadi(exp_sym, sym_var, casadi_var)

    assert(size(sym_var,1)==size(casadi_var,1) && size(sym_var,2)==size(casadi_var,2));

    my_string=char(exp_sym);
    for mi=1:size(sym_var,2) %mi is the mapping index
        x_sym{mi}=sym_var{mi};
        x{mi}=casadi_var{mi};
        for i=1:size(x_sym{mi},1)
            for j=1:size(x_sym{mi},2)
                tmp=['x{' num2str(mi) '}(' num2str(i) ',' num2str(j) ')'];
                my_string=strrep(my_string,char(x_sym{mi}(i,j)),tmp); %replace the string
            end

        end
    end

    exp_casadi=eval(my_string);

end