function cam_pose_w = compute_camera_pose(cam_orientation, f_pts_pose_c, f_pts_pose_w)

    cam_pose_w = zeros(size(f_pts_pose_c));
    max_time_step = size(cam_orientation,1);
    
    for t = 1:max_time_step
        roll = cam_orientation(t,1);
        pitch = cam_orientation(t,2);
        yaw = cam_orientation(t,3);
    
        order = 'rpy';
        Rwc = dcmfromeuler(roll,pitch,yaw,order);
        
        cam_pose_w(t,:) = f_pts_pose_w - f_pts_pose_c(t,:)*Rwc';
    end

end