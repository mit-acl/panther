            %Option 2: Hand-coded. Obtained from running this:
            %syms u real; p=10; order=0; Tmp=(u.^[p:-1:0])'; for (order=0:10) diff(Tmp,u,order)'
            %end
            %This option also saves a lot of comp. time for casadi
%             values_tmp{1}=[u^10, u^9, u^8, u^7, u^6, u^5, u^4, u^3, u^2, u, 1]';
%             values_tmp{2}=[10*u^9, 9*u^8, 8*u^7, 7*u^6, 6*u^5, 5*u^4, 4*u^3, 3*u^2, 2*u, 1, 0]';
%             values_tmp{3}=[90*u^8, 72*u^7, 56*u^6, 42*u^5, 30*u^4, 20*u^3, 12*u^2, 6*u, 2, 0, 0]';
%             values_tmp{4}=[720*u^7, 504*u^6, 336*u^5, 210*u^4, 120*u^3, 60*u^2, 24*u, 6, 0, 0, 0]';
%             values_tmp{5}=[5040*u^6, 3024*u^5, 1680*u^4, 840*u^3, 360*u^2, 120*u, 24, 0, 0, 0, 0]';
%             values_tmp{6}=[30240*u^5, 15120*u^4, 6720*u^3, 2520*u^2, 720*u, 120, 0, 0, 0, 0, 0]';
%             values_tmp{7}=[151200*u^4, 60480*u^3, 20160*u^2, 5040*u, 720, 0, 0, 0, 0, 0, 0]';
%             values_tmp{8}=[604800*u^3, 181440*u^2, 40320*u, 5040, 0, 0, 0, 0, 0, 0, 0]';
%             values_tmp{9}=[1814400*u^2, 362880*u, 40320, 0, 0, 0, 0, 0, 0, 0, 0]';
%             values_tmp{10}=[3628800*u, 362880, 0, 0, 0, 0, 0, 0, 0, 0, 0]';
%             values_tmp{11}=[3628800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]';
% 
%             diffT=values_tmp{order+1}((end-obj.p):end);

%             if(obj.p==2)
%                 if(order==0)
%                     diffT=[u^2 u^1]
%             else if (obj.p==3)
%             
%             else
%                error("Not implemented yet")
%             end