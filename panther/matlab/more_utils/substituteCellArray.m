function cell_array=substituteCellArray(cell_array,tmp_old,tmp_new)

    for i=1:numel(cell_array)
        i
        cell_array{i}=casadi.substitute(cell_array{i},tmp_old,tmp_new);
    end
    
end