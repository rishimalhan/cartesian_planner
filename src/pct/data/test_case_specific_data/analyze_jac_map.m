clear all;
clc;
close all;

jac_map = csvread('jac_mapping.csv');
scatter(jac_map(:,2),jac_map(:,1),'r','filled');
set(gca,'fontsize',30)
set(gcf, 'color', [1,1,1])

title( 'Workspace-C-space Mapping' )
xlabel('Change in X')
ylabel('Change in q')
return;

x = jac_map(1:10:end,1);
y = jac_map(1:10:end,2);

[x,idx] = unique(x);
y = y(idx);

gprMdl1 = fitrgp(x,y,'KernelFunction','exponential',...
              'FitMethod','Exact','PredictMethod','fic','Standardize',1);
[ytestpred,~,ytestci] = predict(gprMdl1,x,'Alpha',0.01);
figure();
% scatter(x,y,'r','filled');
hold on
plot(x,ytestpred,'b');
plot(x,ytestci(:,1),'k:');
plot(x,ytestci(:,2),'k:');
legend('Actual response','GPR predictions',...
'95% lower','95% upper','Location','Best');
hold off