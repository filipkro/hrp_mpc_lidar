%% Generate tractory:


%% Constants

R = 0.1; % Approximation in meters
L = 0.35; % Approximation in meters

% Init

theta = 0;

Br = [R/2 R/2; 0 0; R/(2*L) R/(2*L)];
%Bi = [cos(theta)*R/2 cos(theta)*R/2; sin(theta)*R/2 sin(theta)*R/2;; R/(2*L) -R/(2*L)];

%phi_dot = [ones([1,10]), 2*ones([1,30]) randn([1,1])*ones([1,30]); ones([1,40]), randn([1,1])*ones([1,30])];
%phi_dot = [ones([1,10]), 2*ones([1,10]) randn([1,1])*ones([1,30]); ones([1,40]), randn([1,1])*ones([1,30])];
phi_dot = [1,1]';


m = 10;

for i = 1:m
    phi_dot_add = [abs((randn([1,1]))*ones([1,10])); abs(randn([1,1]))*ones([1,10])];
    phi_dot = [phi_dot, phi_dot_add];
end

n = length(phi_dot);
h = 1;
xi = [0, 0, 0]';
xi_vec = zeros(3,n);


for i = 1:n  
    
   Bi = [cos(theta)*R/2 cos(theta)*R/2; sin(theta)*R/2 sin(theta)*R/2;; R/(2*L) -R/(2*L)];
   xi_dot = Bi*phi_dot(:,i);
   xi = xi + xi_dot*h 
   xi_vec(:,i) = xi;
   theta = xi(3);
   
end


plot(xi_vec(1,:), xi_vec(2,:));
grid on;
lim = 3;
xlim([-lim,lim]);
ylim([-lim,lim]);


%% PATH GENERATION

%path = 