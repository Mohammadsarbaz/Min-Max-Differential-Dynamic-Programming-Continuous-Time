function plot_quadrotor_new(position, orientation_body_wrt_inertia, color, sf)
    % sf: scaling factor
    if nargin < 3
        color = 'green';
        sf = 1;
    end
    
    cylinder_pivot.base_position = sf*[0 0 -0.07];
    cylinder_pivot.base_radius = sf*0.04;
    cylinder_pivot.top_position = sf*[0 0 0.07];
    cylinder_pivot.top_radius = sf*0.04;



    roll = orientation_body_wrt_inertia (1);
    pitch = orientation_body_wrt_inertia (2);
    yaw = orientation_body_wrt_inertia (3);
    order = 'ypr';

    R = dcmfromeuler(roll,pitch,yaw,order);

    lx = sf;
    ly = sf;

    position_rotor_pivot_bc  = R*[-lx/2  0 0]' + position';
    position_rotor_pivot_bc  = position_rotor_pivot_bc';
    %position_rotor_pivot_bc = position + [-lx/2  0 0];

    position_rotor_pivot_uc  = R*[ lx/2  0 0]' + position';
    position_rotor_pivot_uc  = position_rotor_pivot_uc';
    %position_rotor_pivot_uc = position + [ lx/2  0 0];

    position_rotor_pivot_cb  = R*[ 0 -ly/2 0]' + position';
    position_rotor_pivot_cb  = position_rotor_pivot_cb';
    %position_rotor_pivot_cb = position + [ 0 -ly/2 0];

    position_rotor_pivot_cu  = R*[ 0 ly/2 0]' + position';
    position_rotor_pivot_cu  = position_rotor_pivot_cu';
    %position_rotor_pivot_cu = position + [ 0  ly/2 0];

    cone = cylinder_pivot;
    Npoints = 10;


    plot_cone(cylinder_pivot, position_rotor_pivot_bc, orientation_body_wrt_inertia, Npoints, color);
    plot_cone(cylinder_pivot, position_rotor_pivot_uc, orientation_body_wrt_inertia, Npoints, color);
    plot_cone(cylinder_pivot, position_rotor_pivot_cb, orientation_body_wrt_inertia, Npoints, color);
    plot_cone(cylinder_pivot, position_rotor_pivot_cu, orientation_body_wrt_inertia, Npoints, color);

    %%%

    cylinder_rotor.base_position = sf*[0 0 -0.015];
    cylinder_rotor.base_radius = sf*0.14;
    cylinder_rotor.top_position = sf*[0 0 0.015];
    cylinder_rotor.top_radius = sf*0.14;

    position_rotor_bc  = R*[ -lx/2 0 cylinder_pivot.top_position(3)]' + position';
    position_rotor_bc  = position_rotor_bc';
    %position_rotor_bc = position + [-lx/2  0 0];
    %position_rotor_bc(3) = position_rotor_bc(3) + cylinder_pivot.top_position(3);

    position_rotor_uc  = R*[ lx/2 0 cylinder_pivot.top_position(3)]' + position';
    position_rotor_uc  = position_rotor_uc';
    %position_rotor_uc = position + [ lx/2  0 0];
    %position_rotor_uc(3) = position_rotor_uc(3) + cylinder_pivot.top_position(3);

    position_rotor_cb  = R*[ 0 -ly/2 cylinder_pivot.top_position(3)]' + position';
    position_rotor_cb = position_rotor_cb';
    %position_rotor_cb = position + [ 0 -ly/2 0];
    %position_rotor_cb(3) = position_rotor_cb(3) + cylinder_pivot.top_position(3);

    position_rotor_cu  = R*[ 0 ly/2 cylinder_pivot.top_position(3)]' + position';
    position_rotor_cu = position_rotor_cu';
    %position_rotor_cu = position + [ 0  ly/2 0];
    %position_rotor_cu(3) = position_rotor_cu(3) + cylinder_pivot.top_position(3);

    if nargin < 3
        color = 'red';
    end

    plot_cone(cylinder_rotor, position_rotor_bc, orientation_body_wrt_inertia, Npoints, color);
    plot_cone(cylinder_rotor, position_rotor_uc, orientation_body_wrt_inertia, Npoints, color);
    plot_cone(cylinder_rotor, position_rotor_cb, orientation_body_wrt_inertia, Npoints, color);
    plot_cone(cylinder_rotor, position_rotor_cu, orientation_body_wrt_inertia, Npoints, color);


    %%%%

    cylinder_lateral.base_position = [0 0 0];
    cylinder_lateral.base_radius = sf*0.015;
    cylinder_lateral.top_position = [0 0 lx];
    cylinder_lateral.top_radius = sf*0.015;

    position_lateral = position + [-lx/2 0 0];
    %orientation = [0 90 0]/180*pi;


    Rx = dcmfromeuler(0, pi/2, 0,order);
    orientation = R*Rx;

    position_lateral = R*[-lx/2 0 0]' + position';
    position_lateral = position_lateral';

    if nargin < 3
        color = 'blue';
    end

    plot_cone(cylinder_lateral, position_lateral , orientation, Npoints, color);

    cylinder_lateral.base_position = [0 0 0];
    cylinder_lateral.base_radius = sf*0.015;
    cylinder_lateral.top_position = [0 0 ly];
    cylinder_lateral.top_radius = sf*0.015;

    position_lateral = position + [0 -ly/2 0];
    %orientation = [-90 0 0]/180*pi;

    Rx = dcmfromeuler(-pi/2,0,0,order);
    orientation = R*Rx;

    position_lateral = R*[0 -ly/2 0]' + position';
    position_lateral = position_lateral';

    if nargin < 3
        color = 'blue';
    end

    plot_cone(cylinder_lateral, position_lateral , orientation, Npoints, color);



    %%% plotting frame

    cone.base_position = [0 0 0];
    cone.base_radius = sf*0.05;
    cone.top_radius = 0;

    cylinder.base_position = [0 0 0];
    cylinder.base_radius = sf*0.025;
    cylinder.top_radius = sf*0.025;

    offset = sf*[lx/2+0.3 ly/2+0.3 0.3];
    length = sf*[0.5 0.5 0.5];
    body_ratio = [0.8 0.8 0.8];

    in_args.cone = cone;
    in_args.cylinder = cylinder;
    in_args.Npoints = 10;
    in_args.offset = offset;
    in_args.length =length;
    in_args.body_ratio = body_ratio;


    plot_frame (position, orientation_body_wrt_inertia, in_args)
end