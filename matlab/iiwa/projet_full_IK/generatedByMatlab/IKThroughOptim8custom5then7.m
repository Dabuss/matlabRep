function [x,fval,exitflag,output,lambda,grad,hessian] = IKThroughOptim8custom5then7(vrep_store)

FiniteDifferenceStepSize_Data = 0.001*ones(length(vrep_store.tasks)+3,1);
% x0 = zeros(length(vrep_store.tasks)+3,1);
x0 = [3*rand(2,1);rand(length(vrep_store.tasks)+1,1)];
    % x0 = [1;-1.4;-1.8;zeros(length(vrep_store.tasks),1)];
    % x0 = [1.2;-1.3;-2.52;zeros(length(vrep_store.tasks),1)];
    
    
OK = false;
cpt = 0;
while ( ~OK && cpt < 5)
    
    FiniteDifferenceStepSize_Data = 0.001*ones(length(vrep_store.tasks)+3,1);
    
    
    %% Start with the default options
    options = optimoptions('fmincon');
    %% Modify options setting
    options = optimoptions(options,'Display', 'off');
    options = optimoptions(options,'PlotFcn', {  @optimplotx @optimplotconstrviolation });
    options = optimoptions(options,'Algorithm', 'active-set');
    % options = optimoptions(options,'Algorithm', 'interior-point');
    options = optimoptions(options,'FiniteDifferenceStepSize', FiniteDifferenceStepSize_Data);
    [x,fval,exitflag,output,lambda,grad,hessian] = fmincon(@(x)0,x0,[],[],[],[],[],[],@(x)IKConstraints(x,vrep_store,0,0),options);
    x0 = x;
    
    
    c = IKConstraints(x,vrep_store,0,0);
    OK = max(c) < 0;
    cpt = cpt + 1
end




%% Start with the default options
options = optimoptions('fmincon');
%% Modify options setting
options = optimoptions(options,'Display', 'final');
options = optimoptions(options,'PlotFcn', {  @optimplotx @optimplotconstrviolation });
options = optimoptions(options,'Algorithm', 'interior-point');
% options = optimoptions(options,'Algorithm', 'sqp');
options = optimoptions(options,'FiniteDifferenceStepSize', FiniteDifferenceStepSize_Data);
[x,fval,exitflag,output,lambda,grad,hessian] = ...
fmincon(@(x)IKOptimizer(x,vrep_store,10),x0,[],[],[],[],[],[],@(x)IKConstraints(x,vrep_store,0),options)
% [x,fval,exitflag,output,lambda,grad,hessian] = ...
%     fmincon(@(x)sum(IKConstraints(x,vrep_store,0)),x0,[],[],[],[],[],[],@(x)IKConstraints(x,vrep_store,0),options);

c = IKConstraints(x,vrep_store,0);
fval

end