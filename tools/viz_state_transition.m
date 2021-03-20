function fig = viz_state_transition(structs_states, mat_inputs)

% Check if we are missing the final inputs
if size(mat_inputs, 2) == length(structs_states) - 1
    mat_inputs(1, end+1) = 0; % Create new blank column for inputs at final state
elseif size(mat_inputs, 2) ~= length(structs_states)
    error("Input matrix size incompatible.")
end

tab_states = states_structs_to_tab(structs_states);

mat_forces = mat_inputs(1:3, :)';

% Body->World frame transformations
mat_vels_world_frame = rotatepoint(tab_states.quat_body, tab_states.vel_body);
mat_forces_world_frame = rotatepoint(tab_states.quat_body, mat_forces);

fig = figure();
clf;
hold on

plotTransforms(tab_states.posn_body, tab_states.quat_body, 'framesize', 1.8)
quiver3(tab_states.posn_body(:, 1), tab_states.posn_body(:, 2), tab_states.posn_body(:, 3),...
        mat_vels_world_frame(:, 1), mat_vels_world_frame(:, 2), mat_vels_world_frame(:, 3), 'AutoScale','off')

quiver3(tab_states.posn_body(:, 1), tab_states.posn_body(:, 2), tab_states.posn_body(:, 3), ...
        mat_forces_world_frame(:, 1), mat_forces_world_frame(:, 2), mat_forces_world_frame(:, 3), 0.2)

grid on
axis equal

end
