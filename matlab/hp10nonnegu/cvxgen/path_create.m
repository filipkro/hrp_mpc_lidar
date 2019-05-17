function path = path_create(h)

%     t = linspace(1,50,50/h);
%     x = -cos(0.05*t);
%     y = cos(0.1*t).*sin(0.01*t);
%     theta = 0 *t;
%     path = [x;y;theta];
    
        t = linspace(1,100, 100/h);
        theta = 0 *t;
        y = [zeros(1,50/h) 1*ones(1,50/h)];
        %theta = pi/4*ones(1,length(t));
        path = [t;y;theta];
    end