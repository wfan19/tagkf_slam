function [t, structs_states] = sim_state_transition(states_0, mat_inputs, dt, use_ode45)

% Check if optional argument exists
if ~exist("use_ode45", "var")
    use_ode45 = false;
end

structs_states = repelem(states_0, 1, size(mat_inputs, 2) + 1);

t = zeros(1, size(mat_inputs, 2) + 1);

% Generate prediction function handle
if use_ode45
    f_predict = @(state, input, noise, dt)(predict_state_transition_ode45(state, input, noise, dt));
else
    f_predict = @(state, input, noise, dt)(predict_state_transition(state, input, noise, dt));
end

% Run the simulation
for i = 1 : size(mat_inputs, 2)
    t(i+1) = i*dt;
    structs_states(1, i+1) = f_predict(structs_states(1, i), 0, mat_inputs(:, i), dt);
end

end