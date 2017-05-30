function [x,fval,exitflag,output,lambda,grad,hessian] = IKThroughOptim9StepByStep(vrep_store, x0, constraintsToCheck)
%% This is an auto generated MATLAB file from Optimization Tool.

% FiniteDifferenceStepSize_Data = 0.001*ones(length(vrep_store.tasks)+3,1);
FiniteDifferenceStepSize_Data = [0.001,0.001,ones(1,length(vrep_store.tasks)+1)*0.00001];
% x0 = zeros(length(vrep_store.tasks)+3,1);
% x0 = [1;-1.4;-1.8;zeros(length(vrep_store.tasks),1)];
% x0 = [1.2;-1.3;-2.52;zeros(length(vrep_store.tasks),1)];
% x0 = [1.1624   -1.4000   -0.0003    0.0000    0.0000    0.0000    0.0000   -0.0000   -0.0000];

%% Start with the default options
options = optimoptions('fmincon');
%% Modify options setting
options = optimoptions(options,'Display', 'off');
options = optimoptions(options,'PlotFcn', { @optimplotx });
options = optimoptions(options,'Algorithm', 'interior-point');
options = optimoptions(options,'FiniteDifferenceStepSize', FiniteDifferenceStepSize_Data);
% [x,fval,exitflag,output,lambda,grad,hessian] = ...
% fmincon(@(x)IKOptimizer(x,vrep_store,100),x0,[],[],[],[],[],[],@(x)IKConstraintsStepByStep(x,vrep_store,constraintsToCheck),options);
[x,fval,exitflag,output,lambda,grad,hessian] = ...
    fmincon(@(x)IKOptimizer2(x,vrep_store,1),x0,[],[],[],[],[],[],@(x)IKConstraintsStepByStep(x,vrep_store,constraintsToCheck),options);
