%This function returns the matrix that is valid 
%--> for all the segments of a Clamped Uniform BSpline,except for the (deg-1) segments at the beginning and (deg-1) segments at the end 
%--> for all the segments of a NonClamped Uniform BSpline (knots=[a a+delta a+2delta a+3delta,....,a+r*delta, a+(r+1)*delta])
function A=getA_BS(deg, interval)

    A=computeMatrixForNonClampedUniformBSpline(deg,interval);
     
    %segment_key=deg;
    %A=computeMatrixForClampedUniformBSpline(deg,segment_key,interval);

end