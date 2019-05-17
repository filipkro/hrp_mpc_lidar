clear all
close all


h = 1;
reference = path_create(h);

n = floor(length(reference(1,:))/10);

x0 = reference(1,1);
y0 = reference(2,1);
theta0 = 0;
v0 = 0;

params.Q = [100 0 0; 0 100 0; 0 0 0];      %Weight on error
params.Q_final = zeros(3);
params.R = [0.1 0; 0 100];     %Weight on control signal  
params.deltau_max = [2;0.5];  
params.S = 2;           %deltau_max
params.Th = 0.3;

params.u_max = [2;0.1];         %u_max  2x1
 

z = [x0; y0; theta0];
u = [v0; theta0];
vals = {};
for i = 1:length(reference(1,:))-11
    if i > 4
        a = 0;
    end
    params.A = [1 0 -u(1,i)*sin(z(3,i))*h; 0 1 u(1,i)*cos(z(3,i))*h; 0 0 1];
    params.B = [cos(z(3,i))*h -u(1,i)*sin(z(3,i))*0.5*h^2; sin(z(3,i))*h u(1,i)*cos(z(3,i))*0.5*h^2; 0 h];
    params.u_prev = u(:,i);
    params.x_0 = z(:,i);
    params.r_0 = reference(:,1);
    for j = 1:11
        params.r{j} = reference(:,i+j-1);
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

