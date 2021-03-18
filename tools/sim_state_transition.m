function [t, states] = sim_state_transition(states_0, inputs, dt)

states = zeros(size(states_0, 1), size(inputs, 2) + 1);

states(:, 1) = states_0;
t = zeros(1, size(inputs, 2) + 1);

for i = 1 : size(inputs, 2)
    states(:, i + 1) = predict_state_transition(states(:, i), inputs(:, i), 0, dt);
    t(i+1) = i*dt;
end

end