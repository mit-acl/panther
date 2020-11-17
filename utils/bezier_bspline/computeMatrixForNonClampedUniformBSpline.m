function A=computeMatrixForNonClampedUniformBSpline(deg, interval)
   
    %Taken from page 182 of
    %https://link.springer.com/content/pdf/10.1007/s003710050206.pdf  (general matrix representations for B-Splines)
    
    
    Mk=[1]; %this is for k=1 (i.e. deg =2)
    
    for k=2:(deg+1)
      
        Mkm1=Mk;

        Atmp=zeros(k-1,k);
        Btmp=zeros(k-1,k);
        for (i=1:(k-1))
            Atmp(i,i)=i;
            Atmp(i,i+1)=k-i-1;

            Btmp(i,i)=-1;
            Btmp(i,i+1)=1;
        end


        Mk=(1/(k-1))*(  [Mkm1; zeros(1,k-1)]*Atmp   +    [zeros(1,k-1); Mkm1]*Btmp   );
    
    end
    
    %And now change the order of the rows/columns to the convention I use
    A=[];
    for i=1:size(Mk,1) 
        A=[Mk(i,:)'  A];
    end
    
    %A is expressed in t\in[0,1] at this point 
    
    A=convertCoeffMatrixFromABtoCD(A,[0,1],interval);  
    
    % Other way would be to do this
    %     knots=0:15;
    %     segment_index=0; %The result is the same for all the segments
    %     computeMatrixForAnyBSpline(deg,segment_index,knots,interval) %Not
    

end
