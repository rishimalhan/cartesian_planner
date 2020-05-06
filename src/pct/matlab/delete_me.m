clear all;
close all;
clc;

addpath( genpath('/home/rmalhan/Work/USC/Modules/cartesian_planning/cartesian_planner/src/pct/data/csv') );

res = 10;

reach_map = csvread('reach_map.csv');
path_idx = csvread('path_idx.csv');
path_idx = path_idx+1;
path_cost = csvread('path_cost.csv');
pathsToLeaf = csvread('pathsToLeaf.csv');
pathsToLeaf = pathsToLeaf+1;
strt_disc = csvread('strt_discontinuities.csv');

sign=1;
angles(1,1) = 0;
ctr=res;
for j=2:1:size(reach_map,2)
    if (mod(j,2)==0 && j~=2)
        ctr = ctr + res;
    end
    angles(j,1) = sign*ctr;
    sign = sign*-1;
end

reds = [];
greens = [];
figure(1); hold on; 
% daspect([1,1,1]);
for i=1:size(reach_map,1)
    for j=1:1:size(reach_map,2)
        if (reach_map(i,j)==1)
            greens = [greens; i, angles(j,1)];
        else
            reds = [reds; i, angles(j,1)];
        end
    end
end
scatter( greens(:,1),greens(:,2),100,'g','filled' );
scatter( reds(:,1),reds(:,2),100,'r','filled' );
for i=1:size(pathsToLeaf,2)
    scatter( 1:size(pathsToLeaf,1),angles(pathsToLeaf(:,i),1),20,'filled');
end
scatter( 1:size(path_idx,1),angles(path_idx,1),50,'k','filled');



xlabel('Waypoint Number');
ylabel('Angle');
set(gcf,'color','white');
set(gca,'fontweight','bold')
set(gca,'fontsize',15)
title(num2str(path_cost));