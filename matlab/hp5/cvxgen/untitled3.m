clear all
close all


h = 1;
hd = 1;
reference = path_create(h);
Hp = 5;

n = floor(length(reference(1,:))/10);

x0 = reference(1,1);
y0 = reference(2,1);
theta0 = -0.6;
v0 = 0;

params.Q = [100000 0 0; 0 1000 0; 0 0 0.1];      %Weight on error
params.Q_final = zeros(3);
params.R = [0.001 0; 0 100];     %Weight on control signal  
params.deltau_max = [2;0.5];  
params.S = 2;           %deltau_max
params.Th = 0.3;

params.u_max = [2;0.1];         %u_max  2x1
 

z = [x0; y0; theta0];
err = [];
temp1 = [x0; y0; theta0];
u = [v0; theta0];
pose = [x0;y0;theta0];
vals = {};
for i = 1:length(reference(1,:))-Hp
    if i > 24
        z_disc = temp_pos;
        z_disc
        z(:,i)
        A_disc-params.A
        B_disc-params.B
    end
    params.A = [1 0 -u(1,i)*sin(z(3,i))*h; 0 1 u(1,i)*cos(z(3,i))*h; 0 0 1];
    params.B = [cos(z(3,i))*h -u(1,i)*sin(z(3,i))*0.5*h^2; sin(z(3,i))*h u(1,i)*cos(z(3,i))*0.5*h^2; 0 h];
    params.u_prev = u(:,i);
    params.x_0 = z(:,i);
    theta = atan((reference(2,i+Hp)-z(2,i))/(reference(1,i+Hp)-z(1,i)));
    reference(3,i:i+Hp) = theta*ones(1,Hp+1);
    params.r_0 = reference(:,1);
    for j = 1:Hp+1
        params.r{j} = reference(:,i+j-1);
    end
    
    % Exercise the high-speed solver.
    [vars, status] = csolve(params);  % solve, saving results.
    z = [z vars.x{1}];
    u = [u vars.u{1}];
    vals = [vals vars]; 
    % Check convergence, and display the optimal variable value.
    if ~status.converged, error 'failed to converge'; end
    
    z_disc = pose(:,end);
    for j=1:h/hd
        A_disc = [1 0 -u(1,i)*sin(z_disc(3,j))*hd; 0 1 u(1,i)*cos(z_disc(3,j))*hd; 0 0 1];
        B_disc = [cos(z_disc(3,j))*h -u(1,i)*sin(z_disc(3,j))*0.5*hd^2; sin(z_disc(3,j))*hd u(1,i)*cos(z_disc(3,j))*0.5*hd^2; 0 hd];
        temp_pos = A_disc*z_disc(:,j)+B_disc*u(:,i);
        
    end
    if i > 30
        pose(3,end) = 0;
    end
    %pose = [pose z_disc(:,end)];
    temp1 = params.A*temp1+params.B*vars.u{1}
    temp = params.A*pose(:,end)+params.B*vars.u{1};
    pose = [pose temp];
    err = [err z(:,i)-temp1];
end

plot(reference(1,:), reference(2,:));
hold on
%plot(pose(1,:), pose(2,:))
plot(z(1,:),z(2,:))
legend('Reference', 'Lawn mower')
title('Reference following')
xlabel('X-direction')
ylabel('Y-direction')

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
legend('Reference', 'Lawn mower')
title('Step response')
xlabel('Time')
ylabel('Distance in Y-direction')

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

