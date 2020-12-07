
%q and p should be expressed as [real i j k]
%result is also [real i j k]
%Should give the same result as qualmultiply(q,p)
function result= multquat(q,p)

q=q(:);
p=p(:); %Make sure they are column vectors

q0=q(1);
q1=q(2);
q2=q(3);
q3=q(4);

%See eq. C.17 of https://ses.library.usyd.edu.au/handle/2123/19992
%I think https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf has a typo: Eq 109 should be Eq 111 and viceversa
A=[q0 -q1 -q2 -q3;
   q1  q0  -q3  q2;
   q2  q3  q0  -q1;
   q3  -q2  q1  q0;  
];

result=A*p;


end