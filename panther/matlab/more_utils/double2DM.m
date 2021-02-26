function as_DM=double2DM(val)
    if iscell(val)
         tmp={};
        for ii=1:numel(val)
            tmp{ii}=casadi.DM(val{ii});
        end
        as_DM=tmp;
    elseif ismatrix(val)
        tmp=casadi.DM(size(val,1),size(val,2));
        for ii=1:size(val,1)
            for jj=1:size(val,2)
                tmp(ii,jj)=casadi.DM(val(ii,jj));
            end
        end
        as_DM=tmp;
    else
        error("Not implemented yet");
    end
end