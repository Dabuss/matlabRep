function [x,fval,exitflag,output,lambda,grad,hessian] = IKThroughOptim3_OK(vrep_store)
%% This is an auto generated MATLAB file from Optimization Tool.

FiniteDifferenceStepSize_Data = [0.001,0.001,0.001,ones(1,length(vrep_store.tasks))*0.0001]';
x0 = [0;0 ;zeros(length(vrep_store.tasks)+1,1)];

%% Start with the default options
options = optimoptions('fmincon');
%% Modify options setting
options = optimoptions(options,'Display', 'off');
options = optimoptions(options,'PlotFcn', {  @optimplotx @optimplotconstrviolation });
options = optimoptions(options,'FiniteDifferenceStepSize', FiniteDifferenceStepSize_Data);
[x,fval,exitflag,output,lambda,grad,hessian] = ...
fmincon(@(x)0,x0,[],[],[],[],[],[],@(x)IKConstraints(x,vrep_store,0,0,0),options);
