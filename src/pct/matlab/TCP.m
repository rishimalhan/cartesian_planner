%% Tool TCP Computing Function

% Function takes in the Points matrix and Group indices matrix and computes
% the Tool Frame Orientation for each Point.

function [bx,by,bz] = TCP(points,normals)
    % Compute Direction Vector for Y axis
    dir_vec=[];
    by = [];
    bz = [];
    bx = [];
    
   for j=1:size(points,1)
    if j==size(points,1)
    else
       direction = points(j+1,:) - points(j,:);
       dir_vec = direction / norm(direction);
    end
    tool_z = -normals(j,:);
    tool_x = dir_vec;
    tool_y = cross(tool_z,tool_x);
    tool_y = tool_y / norm(tool_y);
    tool_x = cross(tool_y,tool_z);
    tool_x = tool_x / norm(tool_x);
    bx = [bx; tool_x];
    by = [by; tool_y];
    bz = [bz; tool_z];
   end
end