clear all;
close all;
clc;


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