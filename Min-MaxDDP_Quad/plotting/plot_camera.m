function plot_camera (position, orientation, len, focal_dist)
    
    hold on;
    roll = orientation(1);
    pitch = orientation(2);
    yaw = orientation(3);
    order = 'rpy';
    
    R = dcmfromeuler(roll,pitch,yaw,order);
    
    ex_w = [1 0 0]*R + position; 
    ey_w = [0 1 0]*R + position;
    ez_w = [0 0 1]*R + position;
    
%     quiver(position(1),position(2),position(3),...
%            ex_w(1), ex_w(2), ex_w(3),0.5, 'color',[0 0 1]);

%     plot3(pts(:,1),pts(:,2),pts(:,3),...
%            '--rs','LineWidth',2,...
%            'MarkerEdgeColor','k',...
%            'MarkerFaceColor','g',...
%            'MarkerSize',10)
    
    plot3(position(1),position(2),position(3),'o',...
          'MarkerFaceColor','k',...
          'MarkerSize',5); 
      
    plot3(ex_w(1),ex_w(2),ex_w(3),'o',...
          'MarkerFaceColor','y',...
          'MarkerSize',5);
    
    pts = [position; ex_w];
    plot3(pts(:,1),pts(:,2),pts(:,3),...
           '-y','LineWidth',3)  
    
    plot3(ey_w(1),ey_w(2),ey_w(3),'o',...
          'MarkerFaceColor','g',...
          'MarkerSize',5);
    
    pts = [position; ey_w];
    plot3(pts(:,1),pts(:,2),pts(:,3),...
           '-g','LineWidth',3)  
       
    plot3(ez_w(1),ez_w(2),ez_w(3),'o',...
          'MarkerFaceColor','b',...
          'MarkerSize',5);
    
    pts = [position; ez_w];
    plot3(pts(:,1),pts(:,2),pts(:,3),...
           '-b','LineWidth',3) 
       
%     quiver(position(1),position(2),position(3),...
%            ey_w(1), ey_w(2), ey_w(3),0.5, 'color',[0 1 0]);
%        
%     quiver(position(1),position(2),position(3),...
%            ez_w(1), ez_w(2), ez_w(3),0.5, 'color',[1 0 0]);  
    
    lx = len(1);
    ly = len(2);

    verts = [ 0 0 0;
             -0.5*lx -0.5*ly focal_dist;
              0.5*lx -0.5*ly focal_dist;
              0.5*lx  0.5*ly focal_dist;
             -0.5*lx  0.5*ly focal_dist];
    
    verts_w = zeros(5,3);
    for i = 1:5     
        verts_w(i,:) = verts(i,:)*R' + position;     
    end

    faces = [ 1 2 3; ...
              1 2 5; ...
              1 3 4; ...
              1 4 5 ];


    p = patch('Faces',faces, ...
              'Vertices',verts_w, ...
              'FaceColor','y');
    hold on;      
    p = patch('Faces',[2 3 4 5], ...
              'Vertices',verts_w, ...
              'FaceColor','y');   
    hold off;
end