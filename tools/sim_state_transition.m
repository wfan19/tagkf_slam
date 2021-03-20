function [t, structs_states, mat_states] = sim_state_transition(v_states_0, mat_inputs, dt)

mat_states = zeros(size(v_states_0, 1), size(mat_inputs, 2) + 1);

mat_states(:, 1) = v_states_0;
struct_states_0 = state_vec_to_struct(v_states_0);
structs_states = repelem(struct_states_0, 1, size(mat_inputs, 2) + 1);

t = zeros(1, size(mat_inputs, 2) + 1);

for i = 1 : size(mat_inputs, 2)
    
    v_state_next = predict_state_transition(mat_states(:, i), mat_inputs(:, i), 0, dt);
    
    mat_states(:, i+1) = v_state_next;
    
    % Not sure if there's a better way to preallocate than this
    structs_states(1, i+1) = state_vec_to_struct(v_state_next);
    
    t(i+1) = i*dt;
end

end