function [x,fval,exitflag,output,lambda,grad,hessian] = IKThroughOptim7WithObjectiveFunction(x0,FiniteDifferenceStepSize_Data)
%% This is an auto generated MATLAB file from Optimization Tool.

%% Start with the default options
options = optimoptions('fmincon');
%% Modify options setting
options = optimoptions(options,'Display', 'final');
options = optimoptions(options,'PlotFcn', {  @optimplotx @optimplotconstrviolation });
options = optimoptions(options,'Algorithm', 'active-set');
options = optimoptions(options,'FiniteDifferenceStepSize', FiniteDifferenceStepSize_Data);
[x,fval,exitflag,output,lambda,grad,hessian] = ...
fmincon(@(x)IKOptimizer(x,vrep_store,1000),x0,[],[],[],[],[],[],@(x)IKConstraints(x,vrep_store,0),options);
