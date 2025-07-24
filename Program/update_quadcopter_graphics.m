  function update_quadcopter_graphics(hgroup, pos, angles)
        % Check if hgroup is valid
        if ~isvalid(hgroup)
            warning('Invalid graphics handle');
            return;
        end
        
        % Create transformation matrix
        roll = angles(1);
        pitch = angles(2);
        yaw = angles(3);
        
        % Rotation matrices
        Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
        Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
        Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
        
        % Combined rotation
        R = Rz * Ry * Rx;
        
        % Create 4x4 transformation matrix
        T = eye(4);
        T(1:3, 1:3) = R;
        T(1:3, 4) = pos;
        
        % Apply transformation
        set(hgroup, 'Matrix', T);
    end
    