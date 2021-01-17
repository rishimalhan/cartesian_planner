clear all;
close all;
clc;


% part = 'bath_tub';
% part = 'boeing';
part = 'step_slab';
% part = 'fender';

fwd_map = csvread( strcat(part,'/fwd_src_map.csv') );

points = fwd_map(:,1:7);
wp_costs = fwd_map(:,end-1);
q_costs = fwd_map(:,end);

wp_neigh = [];
q_neigh = [];

[val,opt_idx] = min(q_costs);

min(q_costs)
max(q_costs)
mean(q_costs)

% Knn search nXd matrix X and query points Y mXd where d is dimension
kdtree = KDTreeSearcher(points);
for i=1:size(points,1)
    idx = knnsearch(kdtree,points(i,:),'K',10);
%     if ismember(opt_idx,idx)
        wp_neigh = [wp_neigh; wp_costs(i),wp_costs(idx)' ];
%         q_neigh = [q_neigh; q_costs(i),i,idx ];
        q_neigh = [q_neigh; q_costs(i),q_costs(idx)' ];
%     end
end
sortrows(q_neigh,1)
% sortrows(wp_neigh,1)

% wp_costs = (wp_costs-min(wp_costs)) / (max(wp_costs)-min(wp_costs));
% q_costs = (q_costs-min(q_costs)) / (max(q_costs)-min(q_costs));
% 
% scatter( wp_costs,q_costs,20,'filled' )