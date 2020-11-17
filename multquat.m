
%q and p should be expressed as [real i j k]
%result is also [real i j k]
function result= multquat(q,p);

q=q(:);
p=p(:); %Make sure they are column vectors

q0=q(1);
q1=q(2);
q2=q(3);
q3=q(4);

A=[q0 -q1 -q2 -q3;
   q1  q0  -q3  q2;
   q2  q3  q0  -q1;
   q3  -q2  q1  q0;  
];

result=A*p;


end