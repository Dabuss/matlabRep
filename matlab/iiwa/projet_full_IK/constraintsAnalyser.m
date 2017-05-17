function constraintsAnalyser(c,vrep_store)

ndistances = length(vrep_store.distances_handles);
ntasks = length(vrep_store.tasks);
N = (ndistances - 1) + 1 + 1;
%      distances    + jLim + reachability


constraintsNames = cell(length(c),1);
constraintsNames{1} = vrep_store.distances_names{10};

disp(' ')
disp(' ')
disp(' ')

if c(1) > 0
    disp(['         888888', constraintsNames{1}, '   ', num2str(c(1))]);
else
    disp(['               ', constraintsNames{1}, '   ', num2str(c(1))]);
end
        

for i = 1 : ntasks
    constraintsNames{(i-1)*N + 2} = 'Reachability';
    constraintsNames{(i-1)*N + 3} = 'Joint limits';
    constraintsNames(((i-1)*N + 4):((i-1)*N + 4 + ndistances -2)) = vrep_store.distances_names([1:9 11:ndistances]);
    
    disp(['-----  TASK ', num2str(i), ' -----']);
    for j = 1:N
        if c((i-1)*N + j) > 0
            disp(['         888888', constraintsNames{(i-1)*N + j+1}, '   ', num2str(c((i-1)*N + j+1))]);
        else
            disp(['               ', constraintsNames{(i-1)*N + j + 1}, '   ', num2str(c((i-1)*N + j+1))]);
        end
        
    end
    
end

% for i = 2:length(c)
%     
%     indtask = floor((i-1)/ntasks)+1;
%     if mod(i-1,ntasks) == 0
%         disp(['-----  TASK ', num2str(indtask), ' -----']);
%     end
%     disp(['    ',  constraintsNames(i), ' ', num2str(c(i))]);
% end


 
% constraintsNames{(i-1)*N + 2} = ['TASK ',num2str(i),' : reachability'];
%     constraintsNames{(i-1)*N + 3} = ['TASK ',num2str(i),' : joint limits']
end

