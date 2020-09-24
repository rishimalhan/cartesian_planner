clear all;
clc;

figure(1); hold on; daspect([1,1,1])

res = 10;
res = deg2rad(res);
rotm = eye(3);
% rotm(3,3) = rotm(3,3)*-1;
plot_rot(rotm)

tolerances = [ -40,40;
                -40,40;
                0,0];
tolerances = deg2rad(tolerances);
samples = [];
keys = [];
for sam_x = tolerances(1,1):res:tolerances(1,2)
    for sam_y = tolerances(2,1):res:tolerances(2,2)
        for sam_z = tolerances(3,1):res:tolerances(3,2)
            q = eul2quat([sam_z,sam_y,sam_x],'ZYX')+1;
            keys(end+1,:) = int32(q(1)*10)*1e6 + int32(q(2)*10)*1e4 +...
                    int32(q(3)*100)*1e2 + int32(q(4)*10);
            samples(end+1,:) = int32(eul2quat([sam_z,sam_y,sam_x],'ZYX')*10000);
            rot = eul2rotm([sam_z,sam_y,sam_x],'ZYX');
            plot_rot(rot*rotm);
        end
    end
end
size(unique(keys,'rows'))
size(unique(samples,'rows'))

function plot_rot(rot)
    hold on;
    c = ['r','g','b'];
    for i=1:3
        quiver3(0,0,0,rot(1,i),rot(2,i),rot(3,i),c(i));
    end
end