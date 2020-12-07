%q should be expressed as [real i j k]
%q is assummed to be normalized!!!
%Note that this function should return the same as quat2rotm(q)
function R= toRotMat(q)

q=q(:);

q0=q(1);
q1=q(2);
q2=q(3);
q3=q(4);

R=[q0^2+q1^2-q2^2-q3^2  2*q1*q2-2*q0*q3  2*q1*q3+2*q0*q2;
    2*q1*q2+2*q0*q3     q0^2-q1^2+q2^2-q3^2  2*q2*q3-2*q0*q1;
    2*q1*q3-2*q0*q2   2*q2*q3+2*q0*q1  q0^2-q1^2-q2^2+q3^2  
    ];


end