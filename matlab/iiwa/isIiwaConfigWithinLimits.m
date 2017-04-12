function bool = isIiwaConfigWithinLimits(conf)
    
    jointLimits = [170; 120; 170; 120; 170; 120; 175];
    
    n = length(conf);
    bool = true;
    for i=1:n
        angle = conf(i);
        bool = bool & (-jointLimits(i) <= angle && jointLimits(i) >= angle);
        if ~bool
            break;
        end
    end
end