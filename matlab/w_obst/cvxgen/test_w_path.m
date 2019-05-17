clear all
close all


h = 0.1;
%reference = path_create(h);
reference = [linspace(0,20,210); zeros(4,210)];
obst = [10,0];

Hp = 10;

x0 = reference(1,1);
y0 = reference(2,1);
theta0 = 0;
v0 = 0;

params.Q = [1000 0 0 0 0; 0 10 0 0 0; 0 0 1 0 0; 0 0 0 0 0; 0 0 0 0 0];      %Weight on error 3x3
params.Q_final = zeros(5);                      %Final weight   3x3
params.R = [0.1 0; 0 1];     %Weight on control signal          2x2
params.u_max = [2;0.5];         %u_max  2x1
params.deltau_max = [2;0.3];           %deltau_max  2x1
params.dist = [2;2];

z = [x0; y0; theta0; obst(1); obst(2)];

u = [v0; theta0];
vals = {};
for i = 1:length(reference(1,:))-(Hp+1)
    

    params.A = [1 0 -u(1,i)*sin(z(3,i))*h 0 0;...
        0 1 u(1,i)*cos(z(3,i))*h 0 0;...
        0 0 1 0 0;...
        0 0 -u(1,i)*sin(z(3,i))*h 1 0;...
        0 0 u(1,i)*cos(z(3,i))*h 0 1];
    params.B = [cos(z(3,i))*h -u(1,i)*sin(z(3,i))*0.5*h^2;...
        sin(z(3,i))*h u(1,i)*cos(z(3,i))*0.5*h^2;...
        0 h;...
        -cos(z(3,i))*h u(1,i)*sin(z(3,i))*0.5*h^2;...
        -sin(z(3,i))*h -u(1,i)*cos(z(3,i))*0.5*h^2;];
    params.u_prev = u(:,i);
    params.x_0 = z(:,i);
    theta = atan((reference(2,i+Hp)-z(2,i))/(reference(1,i+Hp)-z(1,i)));
    reference(3,i:i+Hp) = theta*ones(1,Hp+1);
    params.r_0 = reference(:,i);
    
    
    
    for j = 1:Hp+1
        params.r{j} = reference(:,i+j);
    end
    % Exercise the high-speed solver.
    [vars, status] = csolve(params);  % solve, saving results.
    z = [z vars.x{1}];
    u = [u vars.u{1}];
    vals = [vals vars];
    
   
    % Check convergence, and display the optimal variable value.
    if ~status.converged, error 'failed to converge'; end
end

plot(reference(1,:), reference(2,:));
hold on
plot(z(1,:),z(2,:))
legend('ref', 'real')

figure(2)
plot(reference(1,:))
hold on
plot(z(1,:))
legend('ref', 'real')
title('X')

figure(3)
plot(reference(2,:))
hold on
plot(z(2,:))
legend('ref', 'real')
title('Y')

% figure(4)
% plot(reference(3,:))
% hold on
% plot(z(3,:))
% legend('ref', 'real')
% title('Theta')

figure(5)
plot(u(1,:))
hold on
plot(u(2,:))
legend('linear', 'angular')

ex = reference(1,1:end-5)-z(1,:);
ey = reference(2,1:end-5)-z(2,:);
norm = norm([ex;ey])

figure(6)
plot(ex)
hold on
plot(ey)
legend('Error X', 'Error Y')
title('Error')

figure(7)
plot(reference(3,:))
hold on
plot(z(3,:))
legend('ref', 'theta')


