clear all;
close all;
clc;


part = 'bath_tub';
% part = 'step_slab';
% part = 'fender';

fwd_map = csvread( strcat(part,'/fwd_src_map.csv') );

points = fwd_map(:,1:7);
wp_costs = fwd_map(:,end-1);
q_costs = fwd_map(:,end);

wp_neigh = [];
q_neigh = [];

% Knn search nXd matrix X and query points Y mXd where d is dimension
kdtree = KDTreeSearcher(points);
for i=1:size(points,1)
    idx = knnsearch(kdtree,points(i,:),'K',10);
    wp_neigh(i,:) = [ wp_costs(i),wp_costs(idx)' ];
    q_neigh(i,:) = [ q_costs(i),q_costs(idx)' ];
end

wp_costs = (wp_costs-min(wp_costs)) / (max(wp_costs)-min(wp_costs));
q_costs = (q_costs-min(q_costs)) / (max(q_costs)-min(q_costs));

scatter( wp_costs,q_costs,20,'filled' )