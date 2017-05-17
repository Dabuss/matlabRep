function [x,fval,exitflag,output,lambda,grad,hessian] = IKThroughOptim5(vrep_store)
%% This is an auto generated MATLAB file from Optimization Tool.
FiniteDifferenceStepSize_Data = 0.001*ones(length(vrep_store.tasks)+3,1);
x0 = zeros(length(vrep_store.tasks)+3,1);
% x0 = [1;-1.4;-1.8;zeros(length(vrep_store.tasks),1)];
% x0 = [1.2;-1.3;-2.52;zeros(length(vrep_store.tasks),1)];
%% Start with the default options
options = optimoptions('fmincon');
%% Modify options setting
options = optimoptions(options,'Display', 'off');
options = optimoptions(options,'PlotFcn', {  @optimplotx @optimplotconstrviolation });
options = optimoptions(options,'Algorithm', 'active-set');
% options = optimoptions(options,'Algorithm', 'interior-point');
options = optimoptions(options,'FiniteDifferenceStepSize', FiniteDifferenceStepSize_Data);
[x,fval,exitflag,output,lambda,grad,hessian] = ...
fmincon(@(x)0,x0,[],[],[],[],[],[],@(x)IKConstraints(x,vrep_store,0,0),options);
