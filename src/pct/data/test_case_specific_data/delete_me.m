clear all;
close all;
clc;

figure
hold on;

% part = 'gear'; opt_Cost = 6.09;
% part = 'step_slab'; opt_Cost = 5.46;
% part = 'bath_tub'; opt_Cost = 4.16;
% part = 'boeing'; opt_Cost = 46.28;
part = 'fender'; opt_Cost = 8.31;

% g_cost_history = csvread( strcat(part,'/random_cost_histories.csv') );
% n_history = csvread( strcat(part,'/random_node_histories.csv') );
% m = getmean(g_cost_history);
% n = getmean(n_history);
% plot(n, m, 'linewidth', 4)

g_cost_history = csvread( strcat(part,'/greedy_cost_histories.csv') );
n_history = csvread( strcat(part,'/greedy_node_histories.csv') );
% n_history = csvread( strcat(part,'/greedy_src_histories.csv') );
m1 = getmean(g_cost_history,false);
max(getmean(g_cost_history,false))
n1 = getmean(n_history,false);

g_cost_history = csvread( strcat(part,'/srcbias_cost_histories.csv') );
n_history = csvread( strcat(part,'/srcbias_node_histories.csv') );
% n_history = csvread( strcat(part,'/srcbias_src_histories.csv') );
m2 = getmean(g_cost_history,false);
n2 = getmean(n_history,false);

m1 = m1 / opt_Cost;
m2 = m2 / opt_Cost;
 
plot(n1, m1, 'linewidth', 4)
plot(n2, m2, 'linewidth', 4)

set(gca,'fontsize',30)
set(gcf, 'color', [1,1,1])

legend( 'Greedy',  'G+B' )
% title('Bath Tub')
return;


% % Code to compute diversity quality
% src = csvread('greedy_source.csv');
% snk = csvread('greedy_sink.csv');
% 
% factor = 1;
% 
% % data = src;
% % src = [];
% % for i=1:size(data,1)
% %     q = [data(i,7),data(i,4:6)];
% %     R = quat2rotm(q);
% %     src(i,:) = [ data(i,1:3),R(:,1)',R(:,2)',R(:,3)'  ];
% % end
% %
% factor = 0.1;
% for i=1:size(points)
%     scatter3( points(:,1),points(:,2),points(:,3),100,cost,'filled' );
%     quiver3( points(:,1),points(:,2),points(:,3),points(:,4),points(:,5),points(:,6),'r',...
%         'AutoScaleFactor', factor);
%     quiver3( points(:,1),points(:,2),points(:,3),points(:,7),points(:,8),points(:,9),'g',...
%         'AutoScaleFactor', factor);
%     quiver3( points(:,1),points(:,2),points(:,3),points(:,10),points(:,11),points(:,12),'b',...
%         'AutoScaleFactor', factor);
% end
% colorbar
% return;
% 
% 
% 
% src = csvread('gd_02_source.csv');
% snk = csvread('gd_02_sink.csv');
% 
% data = src;
% src = [];
% for i=1:size(data,1)
%     q = [data(i,7),data(i,4:6)];
%     R = quat2rotm(q);
%     src(i,:) = [ data(i,1:3),R(:,1)',R(:,2)',R(:,3)'  ];
% end
% 
% quiver3( src(:,1),src(:,2),src(:,3),src(:,4),src(:,5),src(:,6),'r',...
%     'AutoScaleFactor', factor);
% quiver3( src(:,1),src(:,2),src(:,3),src(:,7),src(:,8),src(:,9),'g',...
%     'AutoScaleFactor', factor);
% quiver3( src(:,1),src(:,2),src(:,3),src(:,10),src(:,11),src(:,12),'b',...
%     'AutoScaleFactor', factor);
% 
% return;
% 
% 
% src = csvread('gd_05_source.csv');
% snk = csvread('gd_05_sink.csv');
% 
% score = 0;
% for i=1:size(src,1)
%     for j=i+1:size(src,1)
%         score = score + norm( src(i,:)-src(j,:) );
%     end
% end
% score
% 
% 
% return;



% figure
% hold on;
% 
% opt_path = csvread('../opt_path.csv');
% subopt_path = csvread('opt_path.csv');
% 
% for i=0:3:166
%     nodes = csvread( strcat('wp', num2str(i),'.csv') );
%     dist = vecnorm( nodes - opt_path(i+1,:),2,2 );
%     scatter( ones(size(dist,1),1)*i, dist,80,'b','*' );
%     scatter( i,norm(subopt_path(i+1,:)-opt_path(i+1,:)),100,'m','filled' )
%     plot( [i,i],[0,1.5],'k' );
% end
% set(gca,'fontsize',30)
% set(gcf, 'color', [1,1,1])
% 
% title('WP Opt Source Fix Tube')
% 
% return;







% figure
% hold on;
% 
% opt_path = csvread('../opt_states.csv');
% subopt_path = csvread('joint_states.csv');
% disp( sum(vecnorm(subopt_path(1:end-1,:)-subopt_path(2:end,:),2,2)) );
% 
% % opt_sink_fix: 10.53
% % opt_sink_neigh: 10.46
% % opt_source_fix: 11.10
% % opt_source_neigh: 10.26
% 
% for i=0:3:166
%     nodes = csvread( strcat(num2str(i),'.csv') );
%     dist = vecnorm( nodes - opt_path(i+1,:),2,2 );
%     scatter( ones(size(dist,1),1)*i, dist,80,'b','*' );
%     scatter( i,norm(subopt_path(i+1,:)-opt_path(i+1,:)),100,'m','filled' )
%     plot( [i,i],[0,10],'k' );
% end
% set(gca,'fontsize',30)
% set(gcf, 'color', [1,1,1])
% 
% title('Opt Source Fix Tube')
% 
% return;




figure
hold on;

g_cost_history = csvread('greedy_cost_histories.csv');
n_history = csvread('greedy_node_histories.csv');
n_history = csvread('greedy_src_histories.csv');
m = getmean(g_cost_history);
n = getmean(n_history);
plot(n, m, 'linewidth', 4)

g_cost_history = csvread('gd005_cost_histories.csv');
n_history = csvread('gd005_node_histories.csv');
n_history = csvread('gd005_src_histories.csv');
m = getmean(g_cost_history);
n = getmean(n_history);
plot(n, m, 'linewidth', 4)

g_cost_history = csvread('gd01_cost_histories.csv');
n_history = csvread('gd01_node_histories.csv');
n_history = csvread('gd01_src_histories.csv');
m = getmean(g_cost_history);
n = getmean(n_history);
plot(n, m, 'linewidth', 4)

g_cost_history = csvread('gd5_cost_histories.csv');
n_history = csvread('gd5_node_histories.csv');
n_history = csvread('gd5_src_histories.csv');
m = getmean(g_cost_history);
n = getmean(n_history);
plot(n, m, 'linewidth', 4)

% g_cost_history = csvread('gd_05_cost_histories.csv');
% n_history = csvread('gd_05_node_histories.csv');
% n_history = csvread('gd_05_src_histories.csv');
% m = getmean(g_cost_history);
% n = getmean(n_history);
% plot(n, m, 'linewidth', 4)
% 
% g_cost_history = csvread('gd_1_cost_histories.csv');
% n_history = csvread('gd_1_node_histories.csv');
% n_history = csvread('gd_1_src_histories.csv');
% m = getmean(g_cost_history);
% n = getmean(n_history);
% plot(n, m, 'linewidth', 4)

set(gca,'fontsize',30)
set(gcf, 'color', [1,1,1])
xlabel('N')
ylabel('Cost')

legend('Greedy', 'G+D 005', 'G+D 01', 'G+D 5');

return;



figure
hold on;

g_cost_history = csvread('greedy_cost_histories.csv');
n_history = csvread('greedy_node_histories.csv');
% n_history = csvread('greedy_src_histories.csv');
m = [];
for i=1:size(g_cost_history,2)
    idx = g_cost_history(:,i) < inf;
    m(1,i) = mean(g_cost_history(idx,i));
end
n = mean(n_history,1);
plot(n, m, 'linewidth', 4)



% g_cost_history = csvread('gs_cost_histories.csv');
% n_history = csvread('gs_node_histories.csv');
% n = mean(n_history,1);
% m = [];
% for i=1:size(g_cost_history,2)
%     idx = g_cost_history(:,i) < inf;
%     m(1,i) = mean(g_cost_history(idx,i));
% end
% plot(n, m, 'linewidth', 4)



g_cost_history = csvread('gd_03_cost_histories.csv');
n_history = csvread('gd_03_node_histories.csv');
% n_history = csvread('gd_03_src_histories.csv');
n = mean(n_history,1);
m = [];
for i=1:size(g_cost_history,2)
    idx = g_cost_history(:,i) < inf;
    m(1,i) = mean(g_cost_history(idx,i));
end
plot(n, m, 'linewidth', 4)


% g_cost_history = csvread('gds_05_cost_histories.csv');
% n_history = csvread('gds_05_node_histories.csv');
% n = mean(n_history,1);
% m = [];
% for i=1:size(g_cost_history,2)
%     idx = g_cost_history(:,i) < inf;
%     m(1,i) = mean(g_cost_history(idx,i));
% end
% plot(n, m, 'linewidth', 4)



set(gca,'fontsize',30)
set(gcf, 'color', [1,1,1])
xlabel('N')
ylabel('Cost')

legend('Greedy', 'G+D');

return;






% % Plotting Curves with StdDev
% % g_cost_history = csvread('greedy_cost_histories.csv');
% % n_history = csvread('greedy_node_histories.csv');
% 
% 
% g_cost_history = csvread('random_cost_histories.csv');
% n_history = csvread('random_node_histories.csv');
% 
% m = mean(g_cost_history,1);
% n = mean(n_history,1);
% figure
% hold on;
% 
% plot(n, m, 'linewidth', 4)
% for i=1:size(m,2)
% %     lh = plot( [i*10, i*10], [m(i)-3*sdev(i), m(i)+3*sdev(i)], 'm', 'linewidth', 10 );
% %     lh.Color = [lh.Color 1 / 2];
%     scatter( n(i), max(g_cost_history(:,i)),50,'k','filled' );
%     scatter( n(i), min(g_cost_history(:,i)),50,'k','filled' );
% end
% 
% 
% set(gca,'fontsize',30)
% set(gcf, 'color', [1,1,1])
% xlabel('N')
% ylabel('Cost')
% 
% 
% % title('Greedy')
% title('Random')
% % title('G+S')
% 
% return;




% data = csvread('cost_variation.csv');
% data(data(:,end)>1e5,end) = 20;
% costs = data(:,end);
% data_qt = [];
% for i=1:size(data,1)
%     data_qt(i,:) = [ data(i,1:3),rotm2quat([data(i,4:6)',data(i,7:9)',data(i,10:12)']) ];
% end
% [val,row_id] = min(costs);
% 
% ref = data_qt(row_id,:);
% 
% dist = [];
% for i=1:size(costs,1)
%     dist(i,:) = [ norm(data_qt(i,:)-ref),costs(i) ];
% end
% 
% sortrows(dist,1)
% 
% return;


% figure
% daspect([1,1,1])
% hold on;
% 
% factor = 0.1;
% 
% data = csvread('cost_variation.csv');
% 
% copy_data = data(data(:,end)>1e5,:);
% 
% data(data(:,end)>1e5,:) = [];
% % data(:,end) = data(:,end)*10;
% 
% scatter3( data(:,1),data(:,2),data(:,3),50,data(:,13),'filled' );
% quiver3( data(:,1),data(:,2),data(:,3),data(:,4),data(:,5),data(:,6),'r',...
%     'AutoScaleFactor', factor);
% quiver3( data(:,1),data(:,2),data(:,3),data(:,7),data(:,8),data(:,9),'g',...
%     'AutoScaleFactor', factor);
% quiver3( data(:,1),data(:,2),data(:,3),data(:,10),data(:,11),data(:,12),'b',...
%     'AutoScaleFactor', factor);
% 
% colorbar;
% 
% 
% % return;
% 
% 
% scatter3( copy_data(:,1),copy_data(:,2),copy_data(:,3),50,'r','filled' );
% quiver3( copy_data(:,1),copy_data(:,2),copy_data(:,3),copy_data(:,4),copy_data(:,5),copy_data(:,6),'r',...
%     'AutoScaleFactor', factor);
% quiver3( copy_data(:,1),copy_data(:,2),copy_data(:,3),copy_data(:,7),copy_data(:,8),copy_data(:,9),'g',...
%     'AutoScaleFactor', factor);
% quiver3( copy_data(:,1),copy_data(:,2),copy_data(:,3),copy_data(:,10),copy_data(:,11),copy_data(:,12),'b',...
%     'AutoScaleFactor', factor);
% 
% 
% return;




% cfg1 = csvread('greedyconfigs1.csv');
% cfg2 = csvread('greedyconfigs2.csv');
% cfg3 = csvread('greedyconfigs3.csv');
% cfg4 = csvread('greedyconfigs4.csv');
% 
% pts1 = csvread('greedypoints1.csv');
% pts2 = csvread('greedypoints2.csv');
% pts3 = csvread('greedypoints3.csv');
% pts4 = csvread('greedypoints4.csv');
% 
% ffs = csvread('../neighbors/frames0.csv');
% 
% 
% scatter3( pts(1,1),pts(1,2),pts(1,3),100,'m','filled' );
% quiver3( pts(:,1),pts(:,2),pts(:,3),pts(:,4),pts(:,5),pts(:,6),'r',...
%     'AutoScaleFactor', factor);
% quiver3( pts(:,1),pts(:,2),pts(:,3),pts(:,7),pts(:,8),pts(:,9),'g',...
%     'AutoScaleFactor', factor);
% quiver3( pts(:,1),pts(:,2),pts(:,3),pts(:,10),pts(:,11),pts(:,12),'b',...
%     'AutoScaleFactor', factor);
% 
% 
% 
% return;


% pts = csvread('neighs1.csv');
% 
figure;
hold on;
daspect([1,1,1]);
% 
% factor = 0.5;
% 
% scatter3( pts(1,1),pts(1,2),pts(1,3),100,'m','filled' );
% quiver3( pts(:,1),pts(:,2),pts(:,3),pts(:,4),pts(:,5),pts(:,6),'r',...
%     'AutoScaleFactor', factor);
% quiver3( pts(:,1),pts(:,2),pts(:,3),pts(:,7),pts(:,8),pts(:,9),'g',...
%     'AutoScaleFactor', factor);
% quiver3( pts(:,1),pts(:,2),pts(:,3),pts(:,10),pts(:,11),pts(:,12),'b',...
%     'AutoScaleFactor', factor);
% 
% pause(0.0001)
% 


factor = 1;
pts = csvread('frames1.csv');
quiver3( pts(:,1),pts(:,2),pts(:,3),pts(:,4),pts(:,5),pts(:,6),'r',...
    'AutoScaleFactor', factor);
quiver3( pts(:,1),pts(:,2),pts(:,3),pts(:,7),pts(:,8),pts(:,9),'g',...
    'AutoScaleFactor', factor);
quiver3( pts(:,1),pts(:,2),pts(:,3),pts(:,10),pts(:,11),pts(:,12),'b',...
    'AutoScaleFactor', factor);
% 
% 
return;



points = {};
points{1} = csvread('points0.csv');
points{2} = csvread('points1.csv');
points{3} = csvread('points2.csv');
points{4} = csvread('points3.csv');


figure;
hold on;
daspect([1,1,1]);

for idx=1:5:size(points{1},1)
%     idx = 15;
    line_plot = [];
    for i=1:size(points,2)
        pts = points{i};
    %     area3d(pts(:,1:3));
        quiver3( pts(idx,1),pts(idx,2),pts(idx,3),pts(idx,4),pts(idx,5),pts(idx,6),'r',...
            'AutoScaleFactor', 0.1);
        quiver3( pts(idx,1),pts(idx,2),pts(idx,3),pts(idx,7),pts(idx,8),pts(idx,9),'g',...
            'AutoScaleFactor', 0.1);
        quiver3( pts(idx,1),pts(idx,2),pts(idx,3),pts(idx,10),pts(idx,11),pts(idx,12),'b',...
            'AutoScaleFactor', 0.1);
        line_plot(i,:) = pts(idx,1:3);
    %     cla;
    end
    scatter3( line_plot(1,1),line_plot(1,2),line_plot(1,3),70,'m','filled' );
    plot3( line_plot(:,1),line_plot(:,2),line_plot(:,3),'k','linewidth',3 )
    cla;
end


return;



configs = {};
diff = {};
for i=1:25
%     configs{i} = csvread('configs'+num2str(i)+'.csv');
    cfg1 = csvread( strcat('configs',num2str(i),'.csv') );
    cfg2 = csvread( strcat('configs',num2str(i+1),'.csv') );
    diff{i} = cfg2 - cfg1;
end

jt = 4;
hold on;
for j=1:size(diff{1},1)
    pts = [];
    for i=1:25
        pts = [pts; i,j,diff{i}(j,jt)];
    end
    plot3( pts(:,1),pts(:,2),pts(:,3) )
end

xlabel('Iteration number')
ylabel('Level number')
zlabel('Change in angle')

return;


configs = {};
configs{1} = csvread('configs0.csv');
configs{2} = csvread('configs1.csv');
configs{3} = csvread('configs2.csv');
% configs{4} = csvread('configs3.csv');


points = {};
points{1} = csvread('points0.csv');
points{2} = csvread('points1.csv');
points{3} = csvread('points2.csv');
% points{4} = csvread('points3.csv');


% for i=1:size(configs{1},1)
%     for j=1:4
%         disp(configs{j}(i,:))
%     end
%     fprintf("\n")
% end

figure;
hold on;
daspect([1,1,1]);
for i=1:3
    pts = points{i}(1:3:end,:);
%     area3d(pts(:,1:3));
    quiver3( pts(:,1),pts(:,2),pts(:,3),pts(:,4),pts(:,5),pts(:,6),'r' );
    quiver3( pts(:,1),pts(:,2),pts(:,3),pts(:,7),pts(:,8),pts(:,9),'g' );
    quiver3( pts(:,1),pts(:,2),pts(:,3),pts(:,10),pts(:,11),pts(:,12),'b' );
    cla;
end


return;

opt_config = csvread('fin_config.csv');
config = csvread("init_config.csv");
diff = opt_config - config;
% plot( vecnorm(diff,2,2) )

return;



opt_config = csvread('opt_states.csv');
config = csvread("9.csv");


hash_map = containers.Map;
count = 1;
for i=0:1
    for j=0:1
        for k=0:1
            for l=0:1
                for m=0:1
                    for n=0:1
                        ky = strcat(num2str(i),num2str(j), num2str(k),...
                            num2str(l), num2str(m), num2str(n));
                        hash_map(ky) = count;
                        count = count + 1;
                    end
                end
            end
        end
    end
end

diff = opt_config - config;
key = [];
for i=1:size(diff,1)
    for j=1:6
        if diff(i,j) >= 0
            key(i,j) = 1;
        else
            key(i,j) = 0;
        end
    end
end

class = [];
for i=1:size(key,1)
    k = strcat(num2str(key(i,1)),num2str(key(i,2)), num2str(key(i,3)),...
         num2str(key(i,4)), num2str(key(i,5)), num2str(key(i,6)));
    class(i,1) = hash_map(k);
end

return;


% % Plots for comparing with optimal
% opt_config = csvread('opt_states.csv');
% 
% figure
% hold on;
% for i=9
%     config = csvread(num2str(i)+".csv");
%     dist = vecnorm( opt_config-config,2,2 );
%     plot( dist,'k' );
% end
% 
% config = csvread("closest_configs.csv");
% dist = vecnorm( opt_config-config,2,2 );
% plot( dist,'b' );
% 
% cost = 0;
% dist = [];
% for i=1:size(config,1)-1
%     cost = cost + norm(config(i+1,:)-config(i,:));
%     dist = [dist; norm(config(i+1,:)-config(i,:))];
% end
% cost
% 
% 
% cost = 0;
% dist = [];
% for i=1:size(opt_config,1)-1
%     cost = cost + norm(opt_config(i+1,:)-opt_config(i,:));
%     dist = [dist; norm(opt_config(i+1,:)-opt_config(i,:))];
% end
% cost
% return;




% ratio1 = csvread('bathtub_validity_ratios.csv');
% ratio2 = csvread('stepslab_validity_ratios.csv');
% ratio3 = csvread('boeing_validity_ratios.csv');
% ratio4 = csvread('gear_validity_ratios.csv');
% 
% figure;
% daspect([1,1,1])
% plot( ratio1 )
% title('Bath tub Ratios')
% 
% figure;
% daspect([1,1,1])
% plot( ratio2 )
% title('Step slab Ratios')
% 
% figure;
% daspect([1,1,1])
% plot( ratio3 )
% title('Boeing Ratios')
% 
% figure;
% daspect([1,1,1])
% plot( ratio4 )
% title('Gear Ratios')


c1_opt = csvread('bath_tub_cost_vec.csv');
c2_opt = csvread('step_slab_cost_vec.csv');
c3_opt = csvread('boeing_cost_vec.csv');
c4_opt = csvread('gear_cost_vec.csv');

c1 = csvread('bath_tub_cost_vecs.csv');
c2 = csvread('step_slab_cost_vecs.csv');
c3 = csvread('boeing_cost_vecs.csv');
c4 = csvread('gear_cost_vecs.csv');

% figure
% hold on
% plot( c1_opt, 'k','linewidth',2 ); 
% plot( c1(:,1),'b','linewidth',2 );
% for i=2:9
%     plot( c1(:,i) );
% end
% plot( c1(:,10),'g','linewidth',2 );
% title('Bath Tub')




figure
hold on
plot( c2_opt, 'k','linewidth',2 ); 
plot( c2(:,1),'b','linewidth',2 );
% for i=2:9
%     plot( c2(:,i) );
% end
plot( c2(:,10),'g','linewidth',2 );
title('Step Slab')




% figure
% hold on
% plot( c3_opt, 'k','linewidth',2 ); 
% plot( c3(:,1),'b','linewidth',2 );
% for i=2:9
%     plot( c3(:,i) );
% end
% plot( c3(:,10),'g','linewidth',2 );
% title('Boeing')
% 
% 
% 
% 
% 
% 
% figure
% hold on
% plot( c4_opt, 'k','linewidth',2 ); 
% plot( c4(:,1),'b','linewidth',2 );
% for i=2:9
%     plot( c4(:,i) );
% end
% plot( c4(:,10),'g','linewidth',2 );
% title('Gear')


function m = getmean(g_cost_history,n)
    m = [];
    for i=1:size(g_cost_history,2)-1
        idx = g_cost_history(:,i) < inf;
        m(i,1) = mean(g_cost_history(idx,i));
    end
    if (n)
        max_val = max(m);
        min_val = min(m);
        m = (m-min_val) / (max_val-min_val);
    end
end


function [volume,area]=area3d(v)
% compute volume and area of a convex hull of points v
% Malcolm A. MacIver, 2003
[K volume]=convhulln(v);
%
% Basic formula for computing triangle area
% || = 2-norm, VN = vertex of triangle
% ||V1 X V2 + V2 X V3 + V3 X V1||/2
area= ...
 sum(sqrt(sum(( ...
 [v(K(:,1),2).*v(K(:,2),3) - v(K(:,1),3).*v(K(:,2),2) ...
  v(K(:,1),3).*v(K(:,2),1) - v(K(:,1),1).*v(K(:,2),3)  ...
  v(K(:,1),1).*v(K(:,2),2) - v(K(:,1),2).*v(K(:,2),1)] + ...
 [v(K(:,2),2).*v(K(:,3),3) - v(K(:,2),3).*v(K(:,3),2) ...
  v(K(:,2),3).*v(K(:,3),1) - v(K(:,2),1).*v(K(:,3),3)  ...
  v(K(:,2),1).*v(K(:,3),2) - v(K(:,2),2).*v(K(:,3),1)] + ...
 [v(K(:,3),2).*v(K(:,1),3) - v(K(:,3),3).*v(K(:,1),2) ...
  v(K(:,3),3).*v(K(:,1),1) - v(K(:,3),1).*v(K(:,1),3)  ...
  v(K(:,3),1).*v(K(:,1),2) - v(K(:,3),2).*v(K(:,1),1)]).^2,2))) ...
  /2;
end