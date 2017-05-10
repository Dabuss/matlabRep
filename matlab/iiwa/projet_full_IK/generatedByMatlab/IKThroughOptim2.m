function [x,fval,exitflag,output,lambda,grad,hessian] = IKThroughOptim2(x0,vrep_store)
%% This is an auto generated MATLAB file from Optimization Tool.

FiniteDifferenceStepSize_Data = [0.001,0.001,0.001,ones(1,length(x0)-3)*sqrt(eps)]';

%% Start with the default options
options = optimoptions('fmincon');
%% Modify options setting
options = optimoptions(options,'Display', 'off');
options = optimoptions(options,'PlotFcn', { @optimplotx });
options = optimoptions(options,'FiniteDifferenceStepSize', FiniteDifferenceStepSize_Data);
[x,fval,exitflag,output,lambda,grad,hessian] = ...
fmincon(@(x)0,x0,[],[],[],[],[],[],@(x)IKConstraints(x,vrep_store,0,0,0),options);
