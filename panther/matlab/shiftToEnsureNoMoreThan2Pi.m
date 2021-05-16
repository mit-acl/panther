%Iterate through a vector of angles, and shifts the angles so that the
%difference between two consecutive ones is < than 2pi
function all_phi_correct=shiftToEnsureNoMoreThan2Pi(all_phi)

    all_phi_correct=all_phi(1);

    for phi_i=all_phi(2:end)

%         disp("----")

        previous_phi=all_phi_correct(end);    
        differ=previous_phi-phi_i;    
        phi_i_f=phi_i+floor(differ/(2*pi))*2*pi;
        phi_i_c=phi_i+ceil(differ/(2*pi))*2*pi; 

        if(abs((previous_phi-phi_i_f))<abs((previous_phi-phi_i_c)))
            phi_i_corrected=phi_i_f;
        else
            phi_i_corrected=phi_i_c;
        end

%         fprintf("previous_phi = %f\n", rad2deg(previous_phi))
%         fprintf("phi_i = %f\n", rad2deg(phi_i))
%         fprintf("phi_i_f= %f\n",rad2deg(phi_i_f));
%         fprintf("phi_i_c= %f\n",rad2deg(phi_i_c));
%         fprintf("phi_i_corrected = %f\n", rad2deg(phi_i_corrected))

        all_phi_correct=[all_phi_correct phi_i_corrected];
    end

end