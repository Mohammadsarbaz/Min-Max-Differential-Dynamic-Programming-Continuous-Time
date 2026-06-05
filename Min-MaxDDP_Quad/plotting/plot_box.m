function plot_box (center, len)

    lx = len(1);
    ly = len(2);
    lz = len(3);
    cx = center(1);
    cy = center(2);
    cz = center(3);


    verts = ones(8,1)*[cx cy cz] + ([0 0 0;
                                     1 0 0; 
                                     1 1 0;
                                     0 1 0;
                                     0 1 1;
                                     1 1 1;
                                     1 0 1;
                                     0 0 1]-0.5*ones(8,3)).*(ones(8,1)*[lx ly lz]);

    % There are five faces, defined by connecting the 
    % vertices in the order indicated.
    faces = [ 1 2 3 4; ...
              1 2 7 8; ...
              1 4 5 8; ...
              2 3 6 7; ... 
              3 4 5 6; ...
              5 6 7 8];

    cdata(:,:,1) = [1 0 0 1 1 0 ];
    cdata(:,:,2) = [0 0 0 0 1 1 ];
    cdata(:,:,3) = [1 1 0 0 0 0 ];

    % fvcdata(:,:,1) = [0 0 0 0 1 1 1 1];
    % fvcdata(:,:,2) = [0 0 1 1 0 0 1 1];
    % fvcdata(:,:,3) = [0 1 0 1 0 1 0 1];

    fvcdata = [0 0 1;
               0 0 0;
               1 0 0;
               1 1 0;
               0 1 0;
               0 1 1;
               1 1 1;
               1 0 1];

    cdata = [1 0 1;
               0 0 1;
               0 0 0;
               1 0 0;
               1 1 0;
               0 1 0];      

    p = patch('Vertices',verts, ...
              'Faces',faces, ...
              'FaceColor', 'flat', ...
              'FaceVertexCData', cdata);
    
    hold on
    scatter3(verts(:,1), verts(:,2), verts(:,3),...
          'o', 'filled', ...
          'CData', fvcdata);
    
%     p = patch('Faces',faces, ...
%               'Vertices',verts, ...
%               'Marker','o', ...
%               'MarkerSize', 10, ...
%               'MarkerFaceColor', 'flat', ...
%               'FaceVertexCData', fvcdata,...
%               'EdgeColor','flat', ...
%               'FaceColor','flat');  

    hold off;
end
