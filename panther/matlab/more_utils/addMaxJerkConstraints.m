function constraints=addMaxJerkConstraints(constraints, spline, basis, j_max_scaled)
    for j=1:spline.num_seg
        cps=spline.getCPs_XX_Jerk_ofInterval(basis,j);
        for u=1:size(cps,2)
            for xyz=1:size(cps,1)
                constraints{end+1}=cps{u}(xyz) <= j_max_scaled(xyz) ;
                constraints{end+1}=cps{u}(xyz) >= -j_max_scaled(xyz);
            end
        end
    end
end
