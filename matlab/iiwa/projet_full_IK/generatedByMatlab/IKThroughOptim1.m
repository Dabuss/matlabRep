function [x,fval,exitflag,output,lambda,grad,hessian] = IKThroughOptim1(x0,vrep_store)
%% This is an auto generated MATLAB file from Optimization Tool.

%% Start with the default options
options = optimoptions('fmincon');
%% Modify options setting
options = optimoptions(options,'Display', 'off');
options = optimoptions(options,'PlotFcn', { @optimplotx });
[x,fval,exitflag,output,lambda,grad,hessian] = ...
fmincon(@(x)0,x0,[],[],[],[],[],[],@(x)IKConstraints(x,vrep_store,0,0,0),options);
