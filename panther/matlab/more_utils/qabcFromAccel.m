% /* ----------------------------------------------------------------------------
%  * Copyright 2021, Jesus Tordesillas Torres, Aerospace Controls Laboratory
%  * Massachusetts Institute of Technology
%  * All Rights Reserved
%  * Authors: Jesus Tordesillas, et al.
%  * See LICENSE file for the license information
%  * -------------------------------------------------------------------------- */

function qabc=qabcFromAccel(accel, gravity)

%gravity should be +9.81, or symbolic

ax=accel(1);
ay=accel(2);
az=accel(3);

thrust=[ax ay az+gravity];
thust_normalized=thrust/(sqrt(thrust(1)^2+thrust(2)^2+thrust(3)^2));%norm(thrust); (norm() creates terms like abs()^2 when using symbolic)

a=thust_normalized(1);
b=thust_normalized(2);
c=thust_normalized(3);

qabc=(1/sqrt(2*(1+c)))*[1+c,-b,a,0]; %Note that qabc has norm=1 (because a^2+b^+c^2==1)

end