function tab_tags = tag_arr_to_tab(arr_tags)

% Preallocation
% There will be extra NaN terms that will need to be removed afterwards
secs = nan(length(arr_tags), 1);
ids = nan(length(arr_tags), 1);
corners = nan(4, 2, length(arr_tags));

counter = 1;
% Loop through each tag array message
for i = 1 : length(arr_tags)
    
    % Loop through each element in the tag array message
    for j = 1 : length(arr_tags{i}.Detections)
        detection = arr_tags{i}.Detections(j);
        
        % Save the timestamp
        secs(counter) = detection.Pose.Header.Stamp.Sec + ...
                        detection.Pose.Header.Stamp.Nsec*1e-9;
                    
        % Save the tag id
        ids(counter) = detection.Id;
        
        % Save all the corners
        for k = 1:4
            corners(k, :, counter) = [detection.Corners(k).X, detection.Corners(k).Y];
        end
        
        counter = counter + 1;
    end
    
end

% Remove preallocated NaN elements
secs = secs(~isnan(secs));
ids = ids(~isnan(ids));
corners = corners(:, :, ~all(isnan(corners), [1 2]));

% Extract the individual corner coordinate series from the 3d matrix
bl_corners = corners(1, :, :);
bl_corners = transpose(reshape(bl_corners, [2, size(corners, 3)]));

br_corners = corners(2, :, :);
br_corners = transpose(reshape(br_corners, [2, size(corners, 3)]));

tr_corners = corners(3, :, :);
tr_corners = transpose(reshape(tr_corners, [2, size(corners, 3)]));

tl_corners = corners(4, :, :);
tl_corners = transpose(reshape(tl_corners, [2, size(corners, 3)]));

% Output table
tab_tags = table(secs, ids, bl_corners, br_corners, tr_corners, tl_corners);
end

