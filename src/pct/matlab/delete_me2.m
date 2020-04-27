clear all;
close all;
clc;



noSols_map = csvread('noSols_map.csv');
path_cost = csvread('path_cost.csv');

sign=1;
angles(1,1) = 0;
ctr=1;
for j=2:1:size(noSols_map,2)
    if (mod(j,2)==0 && j~=2)
        ctr = ctr + 5;
    end
    angles(j,1) = sign*ctr;
    sign = sign*-1;
end


figure(1); hold on; daspect([1,1,1]);
for i=1:size(noSols_map,1)
    scatter( ones(size(angles,1),1)*i,angles,50,noSols_map(i,:),'filled' );
end
cb = colorbar;

xlabel('Waypoint Number');
ylabel('Angle');
set(gcf,'color','white');
set(gca,'fontweight','bold')
set(gca,'fontsize',15)
set(cb, 'ticks', 1:max(max(noSols_map)))
title(num2str(path_cost));