function plane()
% Define simulation parameters
t_span = [0,600];          % max time span for simulation 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% state variables Z_1 = x, Z_2 = x_dot
m = 1134; %kg
p = 1.225; %kg/m^3
Aw = 24.0; %m^2
kl = 0.1; %/deg
kdi = 0.001; %deg
kdp = 0.03; 
theta_a = 3; %deg
theta_desired = 2; %deg
Cl = kl * theta_a;
Cd = kdp + kdi * theta_a^2;

bool = true;

% Ft = 1200; % Thrust
% Ft = 1881.2; % Thrust
Ft = 3900; % Thrust

x0 = 0;   % initial displacement
y0 = 0;
vx0 = 0;   % initial velocity, m
vy0 = 0;
count = 1;

Z_0 = [x0, y0, vx0, vy0];     %specify initial conditions (rads, rads/sec)

[t, zout] = ode45(@ode, t_span, Z_0);

function res = ode(t, Z)
res = zeros(4,1);
res(1) = Z(3);
res(2) = Z(4);
v = sqrt(Z(3)^2+Z(4)^2);
theta_c = atan2d(Z(4), Z(3));
theta_ac = theta_c + theta_a;

Fl = 0.5 * p * Cl * Aw * v^2; % Lift
Fd = 0.5;

if Z(2) <= 4100
    Ft  = 3900;  
    theta_a	= 3;
elseif Z(2) <= 4300
    Ft  = 3400;  
    theta_a	= 2.75;
elseif Z(2) <= 4500
    Ft = 2900;
    theta_a = 2.5;
elseif Z(2) <= 4700
    Ft = 2400;
    theta_a = 2.25;
else
    Ft = 1888.157;
    theta_a = 2;
end

theta_ac = theta_c + theta_a;
Fl = 0.5 * p * Cl * Aw * v^2; % Lift
Fd = 0.5 * p * Cd * Aw * v^2; % Drag

res(3)=(Ft*cosd(theta_ac) - Fl*sind(theta_c) - Fd*cosd(theta_c))/m;
res(4)=(-m*9.81 + Ft*sind(theta_ac)+Fl*cosd(theta_c) - Fd*sind(theta_c))/m;

if((Z(2) <= 0) && (res(2) < 0) && (res(4) < 0))
    res(2) = 0;
    res(4) = 0;
end
end

subplot(2,1,1);
hold on
plot(t,zout(:,1))
plot(t,zout(:,2))
legend('x','y')
title(strcat('XY, thrust = ', int2str(Ft)));
xlabel('time') 

subplot(2,1,2); 
hold on
plot(t,zout(:,3))
plot(t,zout(:,4))

legend('vx','vy')
title(strcat(strcat('velocity, angle of attack =  ', int2str(theta_a)), ' deg'));
xlabel('time')

figure
subplot(2,1,1);
hold on;
plot(zout(:,2),zout(:,1));
title('Y v X')

subplot(2,1,2);
hold on;
plot(zout(:,4),zout(:,3));
title('Vy v Vx')
end

