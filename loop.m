function loop()
% Define simulation parameters
t_span = [0,500];          % max time span for simulation 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% state variables Z_1 = x, Z_2 = x_dot
m = 1134; %kg
p = 1.225; %kg/m^3
Aw = 24.0; %m^2
kl = 0.1; %deg
kdi = 0.001; %deg
kdp = 0.03;

level_alt = 2000; % The altitude at which the plane levels off.
R = 80; % m. The radius of the loop
startloop_time = 400; % s. The time spent looping.
endloop_time = 412; % s. The time spent looping.
theta_initial = 3; % Deg. Initial theta taking off.
theta_final = 2; % Deg. Final desired theta.

theta_a = theta_initial; %deg
Cl = kl * theta_a; % Coefficient of lift
Cd = kdp + kdi * theta_a^2; % Coefficient of drag

x0 = 0;   % initial displacement
y0 = 0;
vx0 = 0;   % initial velocity, m
vy0 = 0;

Z_0 = [x0, y0, vx0, vy0];     %specify initial conditions (rads, rads/sec)

[t, zout] = ode45(@ode, t_span, Z_0);

function res = ode(t, Z)
res = zeros(4,1);
res(1) = Z(3);
res(2) = Z(4);
vx = Z(3);
vy = Z(4);
theta_c = atan2d(Z(4), Z(3));
Ft = 3900;

if Z(2) <= (level_alt - 600)
    Ft  = 3900;
    theta_a	= theta_initial;
elseif Z(2) <= (level_alt - 400)
    Ft  = 3000;
    theta_a	= theta_initial + ((theta_final - theta_initial) / 4);
elseif Z(2) <= (level_alt - 200)
    Ft = 2500;
    theta_a = theta_initial + ((theta_final - theta_initial) / 2);
elseif Z(2) <= level_alt
    Ft = 2000;
    theta_a = theta_initial + ((3*(theta_final - theta_initial)) / 4);
else
    Ft = 1888.157;
    theta_a = theta_final;
end

theta_ac = theta_c + theta_a; % New angle of flight
Cl = kl * theta_a; % Coefficient of lift
Cd = kdp + kdi * theta_a^2; % Coefficient of drag

Fl = 0.5 * p * Cl * Aw * (vx^2 + vy^2); % Lift
Fd = 0.5 * p * Cd * Aw * (vx^2 + vy^2); % Drag

if (t < startloop_time || t > endloop_time) 
res(3)=(Ft*cosd(theta_ac) - Fl*sind(theta_c) - Fd*cosd(theta_c))/m;
res(4)=(-m*9.81 + Ft*sind(theta_ac) + Fl*cosd(theta_c) - Fd*sind(theta_c))/m;
end 


if (t >= startloop_time && t <= endloop_time)
    vv = vx^2+vy^2;
    theta_a = (m / (0.5 * p * Aw * kl * vv) * (vv/R + 9.18 *vx /sqrt(vv) ));
    
    Cl = kl * theta_a; % Coefficient of lift
    
    res(3)= -1/(2*m)* (p* sqrt(vv) * Aw* kl *theta_a * vy);
    res(4)=  1/(2*m)* (p* sqrt(vv) * Aw* kl *theta_a * vx)  - 9.81 ;
end

% res(3)=(Ft*cosd(theta_ac) - Fl*sind(theta_c) - Fd*cosd(theta_c))/m;
% res(4)=(-m*9.81 + Ft*sind(theta_ac) + Fl*cosd(theta_c) - Fd*sind(theta_c))/m;

if((Z(2) <= 0) && (res(2) < 0) && (res(4) < 0))
    res(2) = 0;
    res(4) = 0;
end
end

subplot(2,1,1);
hold on
% plot(t,zout(:,1))
plot(t,zout(:,2))
% legend('x','y')
title(strcat('XY, thrust = ', int2str(Ft)));
xlabel('time') 

% subplot(2,1,2); 
% hold on
% plot(t,zout(:,3))
% plot(t,zout(:,4))
% 
% legend('vx','vy')
% title(strcat(strcat('velocity, angle of attack =  ', int2str(theta_a)), ' deg'));
% xlabel('time')
% 
figure
subplot(2,1,1);
hold on;
axis equal
plot(zout(:,1),zout(:,2));
title('Y v X')


subplot(2,1,2);
hold on;
plot(zout(:,4),zout(:,3));
title('Vy v Vx')
end
