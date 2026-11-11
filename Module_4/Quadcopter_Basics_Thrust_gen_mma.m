%% Module 4: Quadcopter Basics - Configuration and Modeling
% =========================================================================
% Author: Ninad Mehta
% Date: 2026-11-11
% Source: MathWorks "Introduction to Quadcopter Modeling & Simulation"
%         Module 4 - Quadcopter Basics
% =========================================================================
%
% THEORY SUMMARY (compressed from the live script)
% -------------------------------------------------
% 1. COORDINATE SYSTEM
%    - Right-hand rule body frame: x_B forward, y_B right, z_B DOWN
%    - Thrust points in -z_B direction (upward)
%
% 2. PROP CONFIGURATIONS
%    - '+' config: props aligned along x/y axes
%    - 'x' config: props offset 45° from x/y axes (more common)
%    - Props 1,3 spin CCW (epsilon=+1); Props 2,4 spin CW (epsilon=-1)
%    - Counter-rotation cancels net yaw torque in hover
%
% 3. INERTIA MODELING
%    - Props modeled as thin disks: I_disk = diag([mr²/4, mr²/4, mr²/2])
%    - Body modeled as cuboid (x-config): 
%      I_body = diag([M(L²+H²)/12, M(L²+H²)/12, ML²/6])
%    - Use parallel axis theorem to shift prop inertia to CoM
%    - For +-config, rotate body inertia by 45° about z-axis
%
% 4. THRUST & TORQUE MODELS (quadratic in omega)
%    T_i = k_T * omega_i²
%    Q_z,i = epsilon_i * k_m * omega_i²
%    where epsilon = [+1, -1, +1, -1]
%
% 5. MIXER MATRIX A
%    Maps [omega_1²; omega_2²; omega_3²; omega_4²] -> [T; Q_x; Q_y; Q_z]
%    A = [ k_T,      k_T,      k_T,      k_T     ;
%         -y_1*k_T, -y_2*k_T, -y_3*k_T, -y_4*k_T ;
%          x_1*k_T,  x_2*k_T,  x_3*k_T,  x_4*k_T ;
%          k_m,     -k_m,      k_m,     -k_m      ]
%    where (x_i, y_i) are prop positions in body frame
%
% 6. HOVER CONDITION
%    At hover: 4*T_prop = M_total * g
%    All moments cancel: Q_x = Q_y = Q_z = 0
% =========================================================================

clear; clc; close all;

%% 1. Define Physical Parameters
% Propeller and body geometry/mass
m     = 0.07;    % propeller mass [kg]
Mbody = 1.02;    % body mass [kg]
L     = 0.17;    % body length in x-y plane [m]
H     = 0.06;    % body height in z direction [m]
Rval  = 0.13;    % arm length (CoM to prop hub) [m]
r     = 0.12;    % propeller radius [m]

% Sanity checks
if Rval <= r
    warning('Rval must be > r for physical feasibility. Adjusting...');
    Rval = r + 0.01;
end
if L >= 2/sqrt(2)*Rval
    warning('Body extends beyond arms. Check geometry.');
end

% Total mass
M = 4*m + Mbody;    % [kg]

% Gravity
g = 9.81;           % [m/s^2]

%% 2. Propeller Positions (symbolic for mixer derivation)
syms R real

% '+' configuration positions
propPlus = [ +R,  0, 0;
              0, +R, 0;
             -R,  0, 0;
              0, -R, 0];

% 'x' configuration positions (45° rotated)
propX = [ R/sqrt(2), -R/sqrt(2), 0;
          R/sqrt(2),  R/sqrt(2), 0;
         -R/sqrt(2),  R/sqrt(2), 0;
         -R/sqrt(2), -R/sqrt(2), 0];

% Select configuration
configuration = "x";
if configuration == "+"
    pB = double(subs(propPlus, R, Rval));
elseif configuration == "x"
    pB = double(subs(propX, R, Rval));
end
disp(['Configuration: ', configuration]);

%% 3. Inertia Calculations
% Propeller inertia about its own center (thin disk)
I_disk_center = diag([ (1/4)*m*r^2, (1/4)*m*r^2, (1/2)*m*r^2 ]);

% Sum propeller inertias about quad CoM using parallel axis theorem
% I_about_CoM = I_center + m*(||p||²*I_3 - p'*p)
I3 = eye(3);
Iprop = zeros(3,3);
for i = 1:4
    p = pB(i,:);
    Iprop = Iprop + (I_disk_center + m*((p*p')*I3 - (p'*p)));
end

% Body inertia (for x-configuration, edges align with axes)
Ibody = diag([(1/12)*Mbody*(L^2+H^2), ...
              (1/12)*Mbody*(L^2+H^2), ...
              (1/12)*Mbody*(L^2+L^2)]);

% For +-configuration, rotate body inertia by 45° about z-axis
if configuration == "+"
    theta = 45;
    RotationMatrix = [cosd(theta) -sind(theta) 0; 
                      sind(theta)  cosd(theta) 0; 
                      0 0 1];
    Ibody = RotationMatrix * Ibody * RotationMatrix';
end

% Total inertia
I = Iprop + Ibody;

disp('Total mass M [kg] =');
disp(M);
disp('Inertia matrix I [kg*m^2] =');
disp(I);

% Extract principal moments
Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);
disp(['Ixx = ', num2str(Ixx), ' kg*m^2']);
disp(['Iyy = ', num2str(Iyy), ' kg*m^2']);
disp(['Izz = ', num2str(Izz), ' kg*m^2']);

%% 4. Thrust and Yaw-Drag Model
% T_i = k_T * omega_i^2
% Q_z,i = epsilon_i * k_m * omega_i^2
% epsilon = [+1, -1, +1, -1] (CCW, CW, CCW, CW)

epsilon = [1, -1, 1, -1];

% Derive mixer matrix A symbolically
syms k_T k_m
pB_sym = propX.';  % Use x-config positions (transposed for column access)

A = [ k_T*ones(1,4);
     -pB_sym(2,:)*k_T;
      pB_sym(1,:)*k_T;
      epsilon*k_m ];

disp('Mixer matrix A (symbolic, x-configuration):');
disp(A);

% Display the mapping: [T; Q_x; Q_y; Q_z] = A * [omega_1^2; omega_2^2; omega_3^2; omega_4^2]
syms omega_1 omega_2 omega_3 omega_4 T Q_x Q_y Q_z
omega = [omega_1^2; omega_2^2; omega_3^2; omega_4^2];
disp('Mapping: [T; Q_x; Q_y; Q_z] = A * omega^2');

%% 5. Estimate Constants from Hover Condition
% At hover: 4 * T_prop = M_total * g
% Assume hover achieved at N = 4000 rpm

W = (Mbody + 4*m) * g;    % Total weight [N]
T_prop = W / 4;            % Thrust per propeller at hover [N]
N = 4000;                  % Nominal hover speed [rpm]

% k_T from T_prop = k_T * N^2  =>  k_T = T_prop / N^2
kT = T_prop / N^2;
disp(['Thrust coefficient kT = ', num2str(kT), ' N/rpm^2']);

% k_m from experimental ratio: k_m / k_T ≈ 10^-2
km = 0.01 * kT;
disp(['Drag coefficient km = ', num2str(km), ' N*m/rpm^2']);

%% 6. Performance Curves - Thrust & Moment vs Speed
N_sweep = 0:1000:10000;    % Sweep 0 to 10,000 rpm
T_sweep = kT * N_sweep.^2;
M_sweep = km * N_sweep.^2;

figure('Name', 'Propeller Performance Curves', 'Position', [100 100 700 500]);

subplot(2,1,1);
plot(N_sweep, T_sweep, 'b-', 'LineWidth', 1.5);
xlabel('Speed (rpm)');
ylabel('Thrust (N)');
title('Thrust vs Speed');
grid minor;

subplot(2,1,2);
plot(N_sweep, M_sweep, 'r-', 'LineWidth', 1.5);
xlabel('Speed (rpm)');
ylabel('Moment (N*m)');
title('Moment vs Speed');
grid minor;

%% 7. Numerical Mixer Matrix and Hover Verification
% Substitute numerical values into mixer matrix A
quad_m = double(subs(A, [k_T, k_m, R], [kT, km, Rval]));

disp('Numerical mixer matrix quad_m:');
disp(quad_m);

% Verify hover condition: all props at N=4000 rpm
N_hover = [4000, 4000, 4000, 4000];
f = quad_m * N_hover'.^2;

disp('Forces and torques at hover (N=4000 rpm each):');
disp(['Total Thrust T = ', num2str(f(1)), ' N  (should equal W = ', num2str(W), ' N)']);
disp(['Q_x = ', num2str(f(2)), ' N*m  (should be ~0)']);
disp(['Q_y = ', num2str(f(3)), ' N*m  (should be ~0)']);
disp(['Q_z = ', num2str(f(4)), ' N*m  (should be ~0)']);

% Check thrust equals weight
thrust_error = f(1) - W;
disp(['Thrust - Weight = ', num2str(thrust_error), ' N  (should be 0)']);

%% 8. Save Parameters for Reuse (Module 5)
% Save all computed parameters to a .mat file for use in QuadcopterMovement
save('quad_params.mat', 'M', 'm', 'Rval', 'r', 'Ixx', 'Iyy', 'Izz', ...
     'quad_m', 'W', 'kT', 'km', 'g');
disp('Parameters saved to quad_params.mat');

