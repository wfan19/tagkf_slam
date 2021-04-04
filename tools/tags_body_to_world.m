function tab_tag_states = tags_body_to_world(tab_states)
% Converts body frame tag positions and orientations to world frame
% positions and orientations

tab_tag_states = table();

%% Calculate world-frame positions
posn_bv_rotated_world_orntn = rotatepoint(tab_states.quat_body, tab_states.posn_bv);
posn_tag_rotated_world_orntn = rotatepoint(tab_states.quat_body, rotatepoint(conj(tab_states.quat_vb), tab_states.posn_tag));

%% Calculate world-frame orientations
quat_tag = tab_states.quat_tag .* tab_states.quat_vb .* tab_states.quat_body;

%% Assign output table states
tab_tag_states.posn_tag = tab_states.posn_body + ...
                posn_bv_rotated_world_orntn + posn_tag_rotated_world_orntn;
            
tab_tag_states.quat_tag = quat_tag;

end

