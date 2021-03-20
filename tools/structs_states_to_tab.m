function tab_states = structs_states_to_tab(structs_states)
% Convert structuer array of state structs to a table
% 
% Individual state vectors are transposed. They are kept as column vectors
% in the state for usage in state transition and update eqns, but here we
% transpose them for usage in the table

names = fieldnames(structs_states);

% Loop over all state structs in the array
for i_struct = 1 : length(structs_states)
    
    % Loop over all the fields in this particular struct
    for i_field = 1 : length(names)
        
        % Transpose this field
        structs_states(i_struct).(names{i_field}) = ...
            transpose(structs_states(i_struct).(names{i_field}));
    end
    
end

tab_states = struct2table(structs_states);

end

