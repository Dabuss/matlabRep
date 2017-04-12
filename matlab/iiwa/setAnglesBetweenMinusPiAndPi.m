function Q1 = setAnglesBetweenMinusPiAndPi(Q)
    for i=1:size(Q,1);
        for j=1:size(Q,2);
            angle  = Q(i,j);
            while (angle <= -pi)
                angle = angle + 2*pi;
            end
            while (angle > pi)
                angle = angle - 2*pi;
            end
            Q1(i,j) = angle;
        end
    end
end