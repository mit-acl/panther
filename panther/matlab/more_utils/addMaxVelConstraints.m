function constraints=addMaxVelConstraints(constraints, spline, basis, v_max_scaled)
    for j=1:spline.num_seg
        cps=spline.getCPs_XX_Vel_ofInterval(basis, j);
        for u=1:size(cps,2)
            for xyz=1:size(cps,1)
                constraints{end+1}=cps{u}(xyz) <= v_max_scaled(xyz);
                constraints{end+1}=cps{u}(xyz) >= -v_max_scaled(xyz);
            end
        end
    end
end


