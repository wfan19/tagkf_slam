function v_states = state_struct_to_vec(struct_states)

% Body states
v_states(1:3) = struct_states.posn_body;
v_states(4:6) = struct_states.vel_body;
v_states(7:10) = struct_states.quat_body;

% Noise incrementing states
v_states(11:13) = struct_states.bias_f;
v_states(14:16) = struct_states.bias_omega;
v_states(17:19) = struct_states.posn_bv;
v_states(20:23) = struct_states.quat_vb;

% Tag states
v_states(24:26) = struct_states.posn_tag;
v_states(27:30) = struct_states.quat_tag;

end

