
function Abs=computeMatrixForClampedUniformBSpline(deg,segment_key,interval)


% *****segment_key= 0 --> Will give the matrix for the segment 0
% *****segment_key= 1 --> Will give the matrix for the segment 1
% *****...
% *****segment_key= deg-2 --> Will give the matrix for the segment deg-2

% *****segment_key > deg-2 --> Will give the matrix for the segments not
% included in the other options (which is the same matrix for all of them)

% *****segment_key= "-(deg-2)" --> Will give the matrix for the segment deg-2

% *****segment_key= -2 --> Will give the matrix for the second to last
% *****segment (antepenultimo);
% *****segment_key= -1 --> Will give the matrix for the last segment

n_int_knots=6;
deltaT=1/(n_int_knots+1);
interm=deltaT*(1:n_int_knots);
knots = [zeros(1,deg+1)   interm   (max(interm)+deltaT)*ones(1,deg+1)];

if(segment_key>=0)
    Abs=computeMatrixForAnyBSpline(deg,deg+1+segment_key,knots,interval);
else
    Abs=computeMatrixForAnyBSpline(deg,size(knots,2)-deg+segment_key,knots,interval);
end
% 
% delta=
% knots=[zeros(1,deg+1), , ones()]
% 
% 
% if(segment_key==0)
% 
% computeMatrixForClampedUniformBSpline
% 
% if(deg==1)
%     Abs=[-1  1 ;
%          1  0 ];
% elseif(deg==2)
%     Abs=(1/2)*[1  -2   1 ;
%             -2   2  1 ;
%              1   0   0  ];
% elseif(deg==3)
%     Abs=(1/6)*[-1 3 -3 1;
%             3 -6 0 4;
%             -3 3 3 1;
%             1 0 0 0 ];
% else
%     error("Not implemented yet")
% end
% 
% 
% 
% if(interval=="m11") %[-1,1]
% 
%    Abs= convertAFrom01toM11(Abs);
% 
% elseif(interval=="01")%[0,1]
%     %Don't do anything
% else
%     error("not implemented yet")
% end
    
end