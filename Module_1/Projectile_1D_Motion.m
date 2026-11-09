% Projectile Motion 1D Code

% 1. Define Parameters
m = 6;          % mass (kg) - not strictly needed for kinematics but good to have
g = 9.81;       % gravitational acceleration (m/s^2)
v0 = 50;        % initial vertical velocity (m/s)
p0 = 10;         % initial position (m) (Note: Simulink instructions said 10, but your code says 0)
dt = 0.01;      % timestep (s)

% 2. Define Time Vector ("some good value")
% Physics check: Time to go up and down is approx 2*v0/g. 
% 2*10/9.81 is about 2 seconds. Let's simulate for 2.5 seconds to be safe.
t_end = 10; 
t = 0:dt:t_end; 
N = length(t);  % Number of steps

% 3. Pre-allocate Arrays (Crucial for MATLAB performance)
% We create empty lists of size N to store velocity and position at every step.
v = zeros(1, N);
p = zeros(1, N);

% Set the starting values (Index 1 is t=0)
v(1) = v0;
p(1) = p0;

% 4. The Loop (This is the simulation engine)
for i = 1:N-1
    % Calculate NEXT velocity based on CURRENT velocity
    % v(i+1) = v(i) - g*dt
    v(i+1) = v(i) - g*dt;

    % Calculate NEXT position based on NEW velocity (Semi-implicit Euler)
    % This matches your screenshot: pNext = p0 + vNext*dt
    p(i+1) = p(i) + v(i+1)*dt;

    % Optional: Stop if it hits the ground (if we started high up)
    % Since p0=0 here, it will go up and come back to 0. 
    % If p0 was 10, we would want to stop when p < 0.
end

% 5. Plotting the Results
figure; % Open a new figure window

% Plot Velocity
subplot(2,1,1); % Top graph
plot(t, v, 'b-', 'LineWidth', 2);
title("Velocity vs Time");
ylabel("Velocity (m/s)");
xlabel("Time (s)");
grid on;

% Plot Position
subplot(2,1,2); % Bottom graph
plot(t, p, 'r-', 'LineWidth', 2);
title("Position vs Time");
ylabel("Displacement (m)");
xlabel("Time (s)");
grid on;

% Solution
tEnd = 2*v0/g

%Exercise 2
% Copy the 1D example and modify the code to solve the 2D problem by 
% properly tracking and displaying both the  and  components of the parabolic path. 

v0 = 50;
angle = 45;
x0 = 0;
y0 = 0;

vy0 = v0*sind(angle)
vx0 = v0*cosd(angle)


%% MAIN EQUATIONS BEING SOLVED:
%
% These are the equations of motion for a projectile under constant gravity:
%
% X-DIRECTION (Horizontal Motion):
%   ax = 0                           (no horizontal acceleration)
%   vx(t) = vx0                      (constant horizontal velocity)
%   x(t)  = x0 + vx0*t               (linear position increase)
%
% Y-DIRECTION (Vertical Motion):
%   ay = -g                          (constant downward acceleration)
%   vy(t) = vy0 - g*t                (velocity decreases linearly)
%   y(t)  = y0 + vy0*t - 0.5*g*t^2   (parabolic position)
dt = 0.01 %timestep (s)

ShowAnimation = true;
tEnd = (vy0+sqrt(vy0^2+2*g*y0))/g;
t = 0:dt:tEnd; % creating time array
TotTimeSteps = length(t);
% vy will CHANGE each timestep (ay=-g) will be updated in for loop. 
vy = zeros(1,TotTimeSteps) % creating empty y-velocity vector

% vx is CONSTANT (no air resistance, ax=0)
vx = vx0*ones(1,TotTimeSteps) % creating constant x-velocity vector

x = zeros(1,TotTimeSteps); % creating empty x position vector
y = zeros(1,TotTimeSteps); % creating empty y position vector

x(1) = x0; % Initial x position
y(1) = y0; % Initial y position
vy(1) = vy0; % Initial vertical velocity

for i = 2:TotTimeSteps
    vy(i) = vy(i-1) - g*dt; % update vertical velocity. % vy = vy0 - g*t  (differential form)
    y(i) = y(i-1) + vy(i)*dt; %update y position using new vel % % y = y0 + ∫vy dt  (numerical integration)
    x(i) = x(i-1) + vx(i)*dt; % update x position with constant vx. % x = x0 + vx0*t  (constant velocity)
end
figure
tl = tiledlayout(2,1);
title(tl,"A Parabolic Path Stopping at 0 on the Descent")
nexttile
plot(t,vy)
ylabel("Velocity $v_y$ (m/s)",Interpreter="latex")
xlabel("Time (s)")

nexttile
plot(x,y)
ylabel('Displacement y (m)')
xlabel('Displacement x (m)')

if ShowAnimation
    figure
    p = scatter(x0,y0,"filled");
    hold on
    pLine = plot(x0,y0);
    hold off
    title("Animation of Projectile Motion")
    grid on
    xlim([min(x)-0.5 max(x)+0.5]) % Set window size to be convenient
    ylim([min(y) max(y)+1])       % for the animation
    for i = 2:(TotTimeSteps)
        p.XData = x(i);
        p.YData = y(i);
        pLine.XData = x(1:i);
        pLine.YData = y(1:i);
        drawnow
    end
end