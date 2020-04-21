clear all;
close all;
clc;

data = csvread('data.csv');

figure(1); hold on;

% Manip
% plot(1:size(data,1)', data(:,1),'b','linewidth',3);
% plot(1:size(data,1)', data(:,4),'g','linewidth',3);

% Dist to UB
plot(1:size(data,1)', data(:,2),'b','linewidth',3);
plot(1:size(data,1)', data(:,5),'g','linewidth',3);

% Dist to LB
plot(1:size(data,1)', data(:,3),'-.b','linewidth',3);
plot(1:size(data,1)', data(:,6),'-.g','linewidth',3);

xlabel('Waypoint Number');
% ylabel('Manipulability');
ylabel('Min Dist to Bounds');
set(gcf,'color','white');
set(gca,'fontweight','bold')
set(gca,'fontsize',15)