% This code allows an interface for the user to select points on a CAD part
% which represent the start and end points for every move. The planner
% converts these points into structured trajectories and writes
% instructions to the robot to follow.

clear all;
close all
clc
dbstop if error;

%% Define part and Plot it

% Define the name of the part stored in the working directory
part = '/home/rmalhan/Work/USC/Modules/cartesian_planning/cartesian_planner/src/pct/data/meshes/bath_tub.stl';
% Note changing direction of Z axis in stl origin. make sure to change the
% dot product in filtering faces in curve_fit

%% Load Part and all the info
%STLREAD is a function obtaiend from matlab exchange. Refer to the file for
%more details.
[v, f, n, name] = stlRead( part );


% Apply Transformation
T = eye(4);
T(1:3,1:3) = eul2rotm([1.57,0,0],'XYZ');

v = ( T * [v' ; ones(1,size(v,1))] )';
v = v(:,1:3);
n = (T(1:3,1:3)*n')';


delete(gca);
close all;    
    
% Choose what color the CAD part needs to be displayed.
col_matrix = [0.941176, 0.972549, 1];

% Plotting the CAD part in Figure-1
figure(1)
patch('Faces',f,'Vertices',v,'FaceVertexCData',col_matrix,'FaceColor',col_matrix);
% set(gca,'zdir','reverse')
xlabel('X-Axis')
ylabel('Y-Axis')
zlabel('Z-Axis')
hold on


%% User-Interface for Selection of Points
  
% Define object for Data Cursor
dcm_obj = datacursormode(figure(1));
set(dcm_obj,'SnapToDataVertex','off')
strt_pts = [];
end_pts = [];
key=0;
flag = 0;
grp=[];
idx = 1;
xyz_bxbybz = [];
daspect([1,1,1])

% Keep Selecting Start and End points alternatively.
while 1
    key=0;
    fprintf('Select Start Point')
    while key==0
        try key = waitforbuttonpress; catch flag=1; break; end 
    end
    if flag==1 
        fprintf('\nSelection Compelte\n'); 
        break; 
    end
    c_info = getCursorInfo(dcm_obj);
    strt_pt = c_info.Position;
    strt_pts = [strt_pts;strt_pt];
    
    key=0;
    fprintf('Select End Point')
    while key==0
        try key = waitforbuttonpress; catch flag=1; break; end
    end
    if flag==1 
        fprintf('\nSelection Complete\n'); 
        break; 
    end
    c_info = getCursorInfo(dcm_obj);
    end_pt = c_info.Position;
    end_pts = [end_pts;end_pt];
    
    [temp_pts,temp_normals,temp_faces] = curve_fit(strt_pt,end_pt,v,f,n);  %Call the curve_fit function
    
    %%%%%%%%%%%%%%%%%%% CAUTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
    %%%%%%%%%%%%%%%%%%% FILTER PRESENT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
    % Filter all points whose Z value is positive. This is to extract
    % points on one side of the part
    indices = find(temp_pts(:,3)>0);
    temp_pts(indices,:) = [];
    temp_normals(indices,:) = [];
    
    % Sort with respect to a axis
    axis = 2; % 1-X, 2-Y, 3-Z
    order = 'ascend';
    temp_pts = sortrows(temp_pts,axis,order);
    
    % Filter points at a distance greater than some resolution
    indices = [];
    for ctr=1:size(temp_pts,1)-1
        if norm(temp_pts(ctr+1,1:3)-temp_pts(ctr,1:3)) < 5/1000
            indices(end+1,1) = ctr+1;
            ctr = ctr + 1;
        end
    end
    temp_pts(indices,:) = [];
    temp_normals(indices,:) = [];
    
    
    %%%%%%%%%%%%%%%%%%% CAUTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
    
    
    %Append the Group Indices
    size_points = size(temp_pts);
    idx_start = idx;
    idx = idx + size_points(1,1)-1;
    idx_end = idx;
    range = [idx_start,idx_end];
    grp = [grp;range]; %Append the Group Indices
    % Appending Group Indices complete
    
    [bx,by,bz] = TCP(temp_pts,temp_normals); %compute_TCP function
    xyz_bxbybz = [xyz_bxbybz; horzcat( temp_pts,bx,by,bz )];
    plot_tcp(bx,by,bz,temp_pts);
    
    %Plot the Points
    scatter3(temp_pts(:,1),temp_pts(:,2),temp_pts(:,3),200,'*','b'); %Plot the Points
    idx = idx + 1;
    daspect([1,1,1])
end
% Temporary tolerance definitions
% tolerances = zeros(size(xyz_bxbybz,1),4);
% tolerances(:,2) = 0.18; % Manually define 7 Degrees tolerance
% tolerances(:,1) = 0.005;
% tolerances(:,3) = 0.0175;
% tolerances(:,4) = 0.0175;
% csvwrite('Ascent_tol_file.csv',tolerances);

if size(xyz_bxbybz,1) > 0
    T = inv(T);
    temp = ( T * [xyz_bxbybz(:,1:3)' ; ones(1,size(xyz_bxbybz,1))] )';
    xyz_bxbybz(:,1:3) = temp(:,1:3);
    xyz_bxbybz(:,4:6) = (T(1:3,1:3)*xyz_bxbybz(:,4:6)')';
    xyz_bxbybz(:,7:9) = (T(1:3,1:3)*xyz_bxbybz(:,7:9)')';
    xyz_bxbybz(:,10:12) = (T(1:3,1:3)*xyz_bxbybz(:,10:12)')';
end

csvwrite('xyz_bxbybz.csv',xyz_bxbybz);
csvwrite('grps.csv',grp);


%% Plot the Tool_TCP

function plot_tcp(bx,by,bz,points)
    bx = bx .* 1;
    by = by .* 1;
    bz = bz .* 1;
    
    hold on
    % Plot the X Vector
    quiver3(points(:,1),points(:,2),points(:,3),...
       bx(:,1),bx(:,2),bx(:,3),'r','linewidth',2,'AutoScaleFactor',0.8,'MaxHeadSize',0.6);
    hold on
    % Plot the Y Vector
    quiver3(points(:,1),points(:,2),points(:,3),...
       by(:,1),by(:,2),by(:,3),'g','linewidth',2,'AutoScaleFactor',0.8,'MaxHeadSize',0.6);
    hold on
    % Plot the Z vector
    quiver3(points(:,1),points(:,2),points(:,3),...
       bz(:,1),bz(:,2),bz(:,3),'b','linewidth',2,'AutoScaleFactor',0.8,'MaxHeadSize',0.6);
end