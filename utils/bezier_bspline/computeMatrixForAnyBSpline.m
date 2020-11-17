function A=computeMatrixForAnyBSpline(deg, index_t_start, knots,interval)

%Everything here is for interval t \in [0,1]

%Following the notation from
%https://link.springer.com/article/10.1007/s003710050206 
%("General matrix representations for B-splines"
% See bottom box page 181

% i=3;

j=index_t_start;

ti=knots(j); tiP1=knots(j+1); tiP2=knots(j+2); tiP3=knots(j+3); tiM1=knots(j-1); tiM2=knots(j-2);

if(deg==1)
    M=[1 0; -1 1];
    %And now change it to the convention I use
    A=[M(2,:)' M(1,:)'];
    
elseif(deg==2)
    
    m00=(tiP1-ti)/(tiP1-tiM1);
    m01=(ti-tiM1)/(tiP1-tiM1);
    m02=0.0;
    
    m10=-2*(tiP1-ti)/(tiP1-tiM1);
    m11=2*(tiP1-ti)/(tiP1-tiM1);
    m12=0.0;
    
    m20=(tiP1-ti)/(tiP1-tiM1);
    m21= -(tiP1-ti)*(   (1/(tiP1-tiM1))  + (1/(tiP2-ti)) );
    m22=(tiP1-ti)/(tiP2-ti); %Note that I think there is a typo in the paper mentioned above in this term (says tiM1 in the denominator, but I think it's ti (if not the row of M doesn't sum 0)
    
    M=[m00 m01 m02;
    m10 m11 m12;
    m20 m21 m22];

    %And now change it to the convention I use
    A=[M(3,:)' M(2,:)' M(1,:)'];

elseif(deg==3)
    
    m00=((tiP1-ti)^2)/((tiP1-tiM1)*(tiP1-tiM2));
    m02=((ti-tiM1)^2)/((tiP2 -tiM1)*(tiP1 -tiM1));
    m01=1-m00-m02;
    m03=0;

    m10=-3*m00; 
    m12 =3*(tiP1-ti)*(ti-tiM1)/((tiP2-tiM1)*(tiP1 -tiM1));
    m11 =3*m00-m12;
    m13=0;

    m20 = 3*m00; 
    m22 = 3*((tiP1-ti)^2)/((tiP2-tiM1)*(tiP1 -tiM1));
    m21 = -3*m00-m22;
    m23= 0;


    m30 = -m00;
    m33= ((tiP1-ti)^2)/((tiP3-ti)*(tiP2-ti));
    m32 = -m22/3 - m33 - ((tiP1-ti)^2)/((tiP2-ti)*(tiP2-tiM1));
    m31 = m00-m32-m33;
    M=[m00 m01 m02 m03;
       m10 m11 m12 m13;
       m20 m21 m22 m23;
       m30 m31 m32 m33];

    %And now change it to the convention I use
    A=[M(4,:)' M(3,:)' M(2,:)' M(1,:)'];
    
else
    error("Not implemented yet")
    
end


%A is expressed in t\in[0,1] at this point 

A=convertCoeffMatrixFromABtoCD(A,[0,1],interval); 

end