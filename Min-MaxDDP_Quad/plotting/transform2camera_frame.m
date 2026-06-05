function f_pts_pose_c = transform2camera_frame (cam_pose_w, cam_orientation_w, f_pts_pose_w)
    
    num_points = size(f_pts_pose_w,1);
    f_pts_pose_c = zeros(num_points,3);
    
    roll = cam_orientation_w(1);
    pitch = cam_orientation_w(2);
    yaw = cam_orientation_w(3);
    
    order = 'rpy';
    Rwc = dcmfromeuler(roll,pitch,yaw,order);

    for i = 1:num_points
        f_pts_pose_c(i,:) = (f_pts_pose_w(i,:) - cam_pose_w)*Rwc;
    end
        
end