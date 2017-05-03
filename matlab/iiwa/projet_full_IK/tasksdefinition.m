
%% init

% Ts = getFkIiwa([0,-20,0,-70,0,0,0]*pi/180,[0,0,0,0,0,0]);
Ts = getFkIiwa([0,-20,0,+70,0,0,0]*pi/180,[0,0,0,0,0,0]);
task_middle = Ts{8};
T = task_middle;

figure(1);clf;
hold on
quiver3(T(1,4),T(2,4),T(3,4),T(1,1),T(2,1),T(3,1),'r'); %ux
quiver3(T(1,4),T(2,4),T(3,4),T(1,2),T(2,2),T(3,2),'g'); %uy
quiver3(T(1,4),T(2,4),T(3,4),T(1,3),T(2,3),T(3,3),'b'); %ux
hold off
xlabel('x')
ylabel('y')
zlabel('z')

vectorscale = 50;

p_offset = [0,50,0,0,0,0]; % x offset of 5 cms


for i = 1 : 10
    T_offset = poseEulerZYXToTransf((i-5)*p_offset);
    T = task_middle * T_offset;
    tasks{i} = T;
    
    %% display
    hold on
    quiver3(T(1,4),T(2,4),T(3,4),T(1,1)*vectorscale,T(2,1)*vectorscale,T(3,1)*vectorscale,'r'); %ux
    quiver3(T(1,4),T(2,4),T(3,4),T(1,2)*vectorscale,T(2,2)*vectorscale,T(3,2)*vectorscale,'g'); %uy
    quiver3(T(1,4),T(2,4),T(3,4),T(1,3)*vectorscale,T(2,3)*vectorscale,T(3,3)*vectorscale,'b'); %ux
    hold off
%     axis equal
    
    
end


axis([-1000,1000,-1000,1000,0,1500])
xlabel('x')
ylabel('y')
zlabel('z')

