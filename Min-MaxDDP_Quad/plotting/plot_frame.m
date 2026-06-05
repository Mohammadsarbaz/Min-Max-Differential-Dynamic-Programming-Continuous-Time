function plot_frame (position, orientation, in_args)

    
    cone = in_args.cone;
    cylinder = in_args.cylinder;

    Npoints = in_args.Npoints;

    offset = in_args.offset;
    length = in_args.length;
    body_ratio = in_args.body_ratio;


    position_from_org_inertia_org_body = position;
    orientation_body_wrt_inertia = orientation;




    position_org_arrowbody_x = [offset(1) 0 0];
    position_org_arrowbody_y = [0 offset(2) 0];
    position_org_arrowbody_z = [0 0 offset(3)];

    position_org_tipx = [offset(1) + length(1)*body_ratio(1) 0 0];
    position_org_tipy = [0 offset(2) + length(2)*body_ratio(2) 0];
    position_org_tipz = [0 0 offset(3) + length(3)*body_ratio(3)];

    roll = orientation_body_wrt_inertia(1);
    pitch = orientation_body_wrt_inertia(2);
    yaw = orientation_body_wrt_inertia(3);
    order = 'ypr';

    R = dcmfromeuler(roll,pitch,yaw,order);


    point_in_inertia = R * position_org_tipx' + position_from_org_inertia_org_body';
    position_org_tipx_in_inertia = point_in_inertia';

    point_in_inertia = R * position_org_arrowbody_x' + position_from_org_inertia_org_body';
    position_org_arrowx_in_inertia = point_in_inertia';


    point_in_inertia = R * position_org_tipy' + position_from_org_inertia_org_body';
    position_org_tipy_in_inertia = point_in_inertia';

    point_in_inertia = R * position_org_arrowbody_y' + position_from_org_inertia_org_body';
    position_org_arrowy_in_inertia = point_in_inertia';


    point_in_inertia = R * position_org_tipz' + position_from_org_inertia_org_body';
    position_org_tipz_in_inertia = point_in_inertia';

    point_in_inertia = R * position_org_arrowbody_z' + position_from_org_inertia_org_body';
    position_org_arrowz_in_inertia = point_in_inertia';

    hold_state = ishold; 
    hold on;

    %%%




    %--- plotting unix e_x vector
    

    Rx = dcmfromeuler(0,pi/2,0,order);
    orientation = R*Rx;


    %--- plotting the body of the arrow
    cylinder.top_position = [0 0 length(1)*body_ratio(1)];
    plot_cone (cylinder, position_org_arrowx_in_inertia , orientation, Npoints, 'red');

    %--- plotting the tip of the arrow
	cone.top_position = [0 0 length(1)*(1-body_ratio(1))];
    plot_cone (cone, position_org_tipx_in_inertia, orientation, Npoints, 'red');


    %--- plotting unix e_y vector
    Ry = dcmfromeuler(-pi/2,0,0,order);
    orientation = R*Ry;

    %--- plotting the body of the arrow
    cylinder.top_position = [0 0 length(2)*body_ratio(2)];
    plot_cone (cylinder, position_org_arrowy_in_inertia, orientation, Npoints, 'green');
   
    %--- plotting the tip of the arrow
    cone.top_position = [0 0 length(2)*(1-body_ratio(2))];
    plot_cone (cone, position_org_tipy_in_inertia, orientation, Npoints, 'green');

    
    %--- plotting unix e_y vector
    orientation = orientation_body_wrt_inertia;

    %--- plotting the body of the arrow
    cylinder.top_position = [0 0 length(3)*body_ratio(3)];
    plot_cone (cylinder, position_org_arrowz_in_inertia, orientation, Npoints, 'blue');

    %--- plotting the tip of the arrow
    cone.top_position = [0 0 length(3)*(1-body_ratio(3))];
    plot_cone (cone, position_org_tipz_in_inertia, orientation, Npoints, 'blue');

%     plot3(position_from_org_inertia_org_body(1), ...
%           position_from_org_inertia_org_body(2), ...
%           position_from_org_inertia_org_body(3), ...
%           'MarkerFaceColor', 'yellow', ...
%           'Marker', 'o', 'MarkerSize', 5);



    if(~hold_state)
        hold off;
    end

end