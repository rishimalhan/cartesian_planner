clear all;
clc;
close all;

figure
hold on;
daspect([1,1,1])

pts1 = csvread('opt_path.csv');
plot_path(pts1, 'k', 0.5 )
pts2 = csvread('opt_source_fix/opt_path.csv');
plot_path(pts2, 'm', 0.5 )



function plot_path(pts, color, factor)
    scatter3( pts(1,1),pts(2,2),pts(3,3),100,'m','filled' );
    scatter3( pts(end,1),pts(end,2),pts(end,3),100,'c','filled' );
    plot3( pts(:,1),pts(:,2),pts(:,3),color,'linewidth',2 );
    quiver3( pts(:,1),pts(:,2),pts(:,3),pts(:,4),pts(:,5),pts(:,6),'r',...
        'AutoScaleFactor', factor);
    quiver3( pts(:,1),pts(:,2),pts(:,3),pts(:,7),pts(:,8),pts(:,9),'g',...
        'AutoScaleFactor', factor);
    quiver3( pts(:,1),pts(:,2),pts(:,3),pts(:,10),pts(:,11),pts(:,12),'b',...
        'AutoScaleFactor', factor);
end