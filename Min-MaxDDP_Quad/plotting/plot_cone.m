function plot_cone (cone, position, orientation, Npoints, color)

    base_position_in_body = cone.base_position;
    base_radius = cone.base_radius;
    top_position_in_body = cone.top_position;
    top_radius = cone.top_radius ;

    %--- orientation can be parameterized as Euler angles or DCM
    if (size(orientation,1) == 1) 
        orientation_body_wrt_inertia = orientation;

        roll = orientation_body_wrt_inertia(1);
        pitch = orientation_body_wrt_inertia(2);
        yaw = orientation_body_wrt_inertia(3);
        order = 'ypr';

        R = dcmfromeuler(roll,pitch,yaw,order);
    else
        R = orientation;
    end
    
    
    theta = linspace(0,2*pi, Npoints);
    point = zeros(1,3);

    %---- begin: computing base circle points in body and inertial frames
    base_circle_points_in_body = zeros(Npoints,3);

    for i = 1:Npoints
        point(1) = base_radius * cos(theta(i));
        point(2) = base_radius * sin(theta(i));

        base_circle_points_in_body(i,:) = point + base_position_in_body;
    end
    
    position_from_org_inertia_to_org_body = position;
    
    base_circle_points_in_inertia = zeros(Npoints,3);

    for i = 1:Npoints
        point_in_body = base_circle_points_in_body(i,:);

        point_in_inertia = R * point_in_body' + position_from_org_inertia_to_org_body';
        base_circle_points_in_inertia(i,:) = point_in_inertia';
    end
    %---- end: computing base circle points in body and inertial frames
    
    %---- begin: computing top circle points (point) in body and inertial frames
    if (top_radius > 0)
        point = zeros(1,3);
        top_circle_points_in_body = zeros(Npoints,3);
        
        for i = 1:Npoints
            point(1) = top_radius * cos(theta(i));
            point(2) = top_radius * sin(theta(i));

            top_circle_points_in_body(i,:) = point + top_position_in_body;
        end
        
        top_circle_points_in_inertia = zeros(Npoints,3);
        
        for i = 1:Npoints
            point_in_body = top_circle_points_in_body(i,:);

            point_in_inertia = R * point_in_body' + position_from_org_inertia_to_org_body';
            top_circle_points_in_inertia(i,:) = point_in_inertia';
        end
    else
        top_point_in_body = top_position_in_body;
        point_in_body = top_point_in_body ;
        
        point_in_inertia = R * point_in_body' + position_from_org_inertia_to_org_body';
        top_point_in_inertia = point_in_inertia';
    end
    %---- end: computing top circle points (point) in body and inertial frames

    
    hold_state = ishold; hold on;
    
    %--- plotting base circle
    vertices = base_circle_points_in_inertia;
    faces = 1:Npoints;

    patch('Vertices', vertices, 'Faces', faces, 'FaceColor',color);


    if (top_radius > 0)
        %--- plotting top circle
        vertices = top_circle_points_in_inertia;
        faces = 1:Npoints;

        patch('Vertices', vertices, 'Faces', faces, 'FaceColor',color);
        
        %--- plotting surface
        vertices = [base_circle_points_in_inertia; top_circle_points_in_inertia];
        faces = zeros(Npoints-1,4);

        for i = 1:Npoints-1
            faces (i,:) = [i, i+1, i+1+Npoints, i+Npoints];
        end

        patch('Vertices', vertices, 'Faces', faces, 'FaceColor',color, ...
            'LineStyle', 'none');
    else
         %--- plotting surface
        vertices = [base_circle_points_in_inertia; top_point_in_inertia];
        
        faces = zeros(Npoints-1,3);

        for i = 1:Npoints-1
            faces (i,:) = [i, i+1, Npoints+1];
        end

        patch('Vertices', vertices, 'Faces', faces, 'FaceColor',color, ...
            'LineStyle', 'none'); 
    end
    
    %---

    
    if(~hold_state)
        hold off;
    end
end