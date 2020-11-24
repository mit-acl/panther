function result=getSimpsonCoeff(j,total)

    j_is_even = (rem(j, 2) == 0);

    %https://en.wikipedia.org/wiki/Simpson%27s_rule
    if(j==1 || j==total)  %Beginning or end
        result=1.0;
    elseif (j_is_even)
        result=4.0;
    else 
        result=2.0;
    end

end