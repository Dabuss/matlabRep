function [x,fval,exitflag,output,lambda,grad,hessian] = IKThroughOptim12ForceCapa(vrep_store)
%% This is an auto generated MATLAB file from Optimization Tool.
FiniteDifferenceStepSize_Data = [0.0001*ones(3,1); 0.01*ones(length(vrep_store.tasks),1)];
% FiniteDifferenceStepSize_Data = [0.001,0.001,ones(1,length(vrep_store.tasks)+1)*0.00001];
% x0 = zeros(length(vrep_store.tasks)+3,1);
x0 = [1.2;-1.4;0;zeros(length(vrep_store.tasks),1)];
% x0 = [1.2;-1.3;-2.52;zeros(length(vrep_store.tasks),1)];
% x0 = [1.1624   -1.4000   -0.0003    0.0000    0.0000    0.0000    0.0000   -0.0000   -0.0000];

lb = [0,-1.652,-pi, -pi*ones(1,length(vrep_store.tasks))];
ub = [2.1833,-0, pi, pi*ones(1,length(vrep_store.tasks))];

f = [0;0;-400;0;0;0];

%% Start with the default options
options = optimoptions('fmincon');
%% Modify options setting
options = optimoptions(options,'Display', 'off');
options = optimoptions(options,'PlotFcn', {  @optimplotx});
options = optimoptions(options,'Algorithm', 'sqp');
% options = optimoptions(options,'Algorithm', 'interior-point');
options = optimoptions(options,'FiniteDifferenceStepSize', FiniteDifferenceStepSize_Data);
% [x,fval,exitflag,output,lambda,grad,hessian] = fmincon(@(x)IKOptimizer(x,vrep_store,10),x0,[],[],[],[],lb,ub,@(x)IKConstraints(x,vrep_store,0,0),options);
[x,fval,exitflag,output,lambda,grad,hessian] = fmincon(@(x)0,x0,[],[],[],[],lb,ub,@(x)IKConstraints(x,vrep_store,f),options);
% [x,fval,exitflag,output,lambda,grad,hessian] = fmincon(@(x)IKOptimizer2(x,vrep_store,10),x0,[],[],[],[],lb,ub,@(x)IKConstraints(x,vrep_store,0,0),options);