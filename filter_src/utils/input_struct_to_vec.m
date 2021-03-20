function v_inputs = input_struct_to_vec(struct_inputs)

v_inputs = [struct_inputs.f(:); struct_inputs.omega(:)];

end
