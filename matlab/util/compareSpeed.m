function compareSpeed(nIter, func1, x1, func2, x2)
% util function to quicky compare the speed between two functions

% usecase : compareSpeed(1000,@norm,[1,1],@(x)sqrt(x(1).*x(1)+x(2).*x(2)),[1,1])

f1 = @(x)func1(x);
f2 = @(x)func2(x);


tic
for i=1:nIter
    f1(x1);
end
t1 = toc;
disp(['function 1 took ', num2str(t1), ' seconds for ', num2str(nIter), ' iterations']);

tic
for i=1:nIter
    f2(x2);
end
t2 = toc;
disp(['function 2 took ', num2str(t2), ' seconds for ', num2str(nIter), ' iterations']);

end

