function struct_inputs = input_vec_to_struct(v_inputs)

struct_inputs = struct();

struct_inputs.f = v_inputs(1:3);
struct_inputs.omega = v_inputs(4:6);

end

