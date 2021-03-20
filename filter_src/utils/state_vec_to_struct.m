function struct_states = state_vec_to_struct(v_states)

% Force v_states to be a column vector
v_states = v_states(:);

struct_states = struct();

% Body states
struct_states.posn_body = v_states(1:3);
struct_states.vel_body = v_states(4:6);
struct_states.quat_body = quaternion(v_states(7:10)');

% Noise incrementing states
struct_states.bias_f = v_states(11:13);
struct_states.bias_omega = v_states(14:16);
struct_states.posn_bv = v_states(17:19);
struct_states.quat_vb = quaternion(v_states(20:23)');

% Tag states
struct_states.posn_tag = v_states(24:26);
struct_states.quat_tag = quaternion(v_states(27:30)');

end

