function [t, structs_states, mat_states] = sim_state_transition(v_states_0, mat_inputs, dt, use_ode45)

% Check if optional argument exists
if ~exist("use_ode45", "var")
    use_ode45 = false;
end
    
mat_states = zeros(size(v_states_0, 1), size(mat_inputs, 2) + 1);

mat_states(:, 1) = v_states_0;
struct_states_0 = state_vec_to_struct(v_states_0);
structs_states = repelem(struct_states_0, 1, size(mat_inputs, 2) + 1);

t = zeros(1, size(mat_inputs, 2) + 1);

% Generate prediction function handle
if use_ode45
    f_predict = @(states, inputs, noise, dt)(predict_state_transition_ode45(states, inputs, noise, dt));
else
    f_predict = @(states, inputs, noise, dt)(predict_state_transition(states, inputs, noise, dt));
end

for i = 1 : size(mat_inputs, 2)
    
    v_state_next = f_predict(mat_states(:, i), 0, mat_inputs(:, i), dt);
    
    mat_states(:, i+1) = v_state_next;
    
    % Not sure if there's a better way to preallocate than this
    structs_states(1, i+1) = state_vec_to_struct(v_state_next);
    
    t(i+1) = i*dt;
end

end