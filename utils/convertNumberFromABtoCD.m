% /* ----------------------------------------------------------------------------
%  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
%  * Massachusetts Institute of Technology
%  * All Rights Reserved
%  * Authors: Jesus Tordesillas, et al.
%  * See LICENSE file for the license information
%  * -------------------------------------------------------------------------- */

%x expressed in [a,b] --> x expressed in [c,d]
function x_converted=convertNumberFromABtoCD(x,ab_interval,cd_interval)

a=ab_interval(1);
b=ab_interval(2);
c=cd_interval(1);
d=cd_interval(2);


if(b<=a)
    error("Need b>a")
end

if(d<=c)
    error("Need d>c")
end

x_converted=((d-c)/(b-a))*(x-a) + c;

%x_converted=((b-a)/(d-c))*(x-c)+a;

end