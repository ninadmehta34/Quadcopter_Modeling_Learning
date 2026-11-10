% Module 3: Simple Quadcopter Propeller Model
% =========================================================================
% Author: Ninad Mehta
% Date: 2026-11-10
% Source: MathWorks "Introduction to Quadcopter Modeling & Simulation"
%         Module 3 - Propeller Model
% =========================================================================
%
% PURPOSE
% -------
% Build a simple, physically meaningful propeller model that converts
% motor RPM into thrust and reaction torque. This is the fundamental
% actuator model the quadcopter plant will use.
%
% THEORY SUMMARY (compressed from the live script)
% -------------------------------------------------
% 1. MOTIVATION
%    A propeller converts electrical energy (motor) into mechanical thrust.
%    We want the simplest model that respects units and scaling. Complexity
%    (blade twist, compressibility, etc.) can be added later.
%
% 2. MOMENTUM THEORY (global approach)
%    Model the propeller as an actuator disk that accelerates air downward.
%      - Mass flow rate:    m_dot = rho * A * v = rho * pi * r^2 * v
%      - Induced velocity:  v is proportional to blade tip speed (omega*r)
%      - Thrust (Newton's 2nd law): T = m_dot * Delta_v
%      - Result: T ∝ rho * pi * r^4 * omega^2
%    Key insight: thrust has FOURTH-ORDER dependence on propeller radius.
%    Doubling the radius gives ~16x the thrust (at same omega).
%
% 3. BLADE ELEMENT REASONING (local approach, confirms momentum theory)
%    Look at one small blade section at radius x:
%      - Local tangential speed: v(x) = omega * x
%      - Dynamic pressure:       q = 0.5 * rho * omega^2 * x^2
%      - Incremental thrust:     dT ∝ q * c(x) * dx
%        where c(x) is chord width. For geometrically similar blades,
%        c(x) ∝ r (constant ratio c/r along the blade).
%      - Integrate from 0 to r:  T ∝ rho * omega^2 * r^4   ✓ matches
%
% 4. DIMENSIONAL ANALYSIS (third confirmation)
%    Thrust has units [kg*m/s^2]. The only way to combine:
%      - rho   [kg/m^3]
%      - omega [1/s]
%      - r     [m]
%    to get force units is: T ∝ rho * r^4 * omega^2
%    Define dimensionless coefficient C_T so that:
%      T = C_T * rho * r^4 * omega^2
%
% 5. MODELING ASSUMPTIONS (validity domain)
%    - Rigid, fixed-pitch blades (geometry constant)
%    - Incompressible flow (rho constant)
%    - Thrust acts along propeller axis
%    - No blade flapping, no induced velocity modeling
%    Valid for: subsonic flow, hover/slow flight, fixed-pitch props,
%               moderate Reynolds numbers.
%
% 6. THRUST MODEL (final form)
%      T = k_T * omega^2
%    where k_T = C_T * rho * r^4   [N/(rad/s)^2]
%    Physical interpretation: faster rotation → higher tip speed →
%    larger dynamic pressure → quadratic thrust growth.
%
% 7. REACTION (DRAG) TORQUE MODEL
%    Aerodynamic drag on the blades produces a reaction torque on the
%    motor shaft opposing rotation. By similar blade-element analysis:
%      dQ = x * dD,  where dD ∝ rho * omega^2 * x^2 * c(x) * C_Q * dx
%    Integrating:  Q ∝ C_Q * omega^2 * r^5
%    Final form:
%      Q = k_m * omega^2
%    where k_m includes C_Q and r^5   [N*m/(rad/s)^2]
%    Power required: P = omega * Q ∝ omega^3
%
% 8. IMPACT ON QUADCOPTER DESIGN
%    - Torque acts about the rotation axis → causes YAW motion
%    - Balanced operation requires PAIRS of counter-rotating propellers
%    - Encode rotation direction with sign epsilon = ±1:
%          Q_z = epsilon * k_m * omega^2
%      epsilon = +1 for CCW, -1 for CW
%
% KEY TAKEAWAY FOR THE PLANNER
% ----------------------------
% Input:  omega [rad/s]  (what the motor controller commands)
% Output: T [N] and Q [N*m]  (what the plant dynamics use)
% The planner only needs to know: higher omega → more thrust AND more
% reaction torque. The exact coefficients (k_T, k_m) come from the
% specific propeller geometry and air density.
% =========================================================================

clear; clc; close all;

%% 1. Define Propeller Parameters
% These are example values for a small quadcopter propeller.
% In practice, k_T and k_m are measured experimentally or computed from
% blade geometry + airfoil data.

kT      = 1.8e-5;     % Thrust coefficient        [N/(rad/s)^2]
km      = 2.0e-7;     % Drag (moment) coefficient   [N*m/(rad/s)^2]
spinDir = 1;          % +1 = CCW, -1 = CW

% Sweep range for plotting
omega = linspace(0, 900, 200);   % rotational speed [rad/s]

%% 2. Compute Thrust and Torque
% Formulas from SimplePropellerModel:
%   T = kT * omega^2
%   Q = spinDir * km * omega^2
%
% NOTE: Thrust grows as omega^2, and torque also grows as omega^2.
% However, kT >> km typically, so thrust magnitude dominates numerically.
% The ratio T/Q = kT/km is roughly constant for a given propeller.

[T,Q] = SimplePropellerModel(omega, kT, km, spinDir);


%% 3. Plot Results
figure('Name', 'Simple Propeller Model: Thrust and Drag Torque', ...
       'Position', [100 100 700 400]);

yyaxis left
plot(omega, T, 'b-', 'LineWidth', 1.5);
ylabel('Thrust [N]', 'Color', 'b');

yyaxis right
plot(omega, Q, 'r--', 'LineWidth', 1.5);
ylabel('Torque [N\cdotm]', 'Color', 'r');

xlabel('\omega [rad/s]');
title('Simple Propeller Model: Thrust and Drag Torque vs. Rotational Speed');
legend('Thrust T', 'Torque Q', 'Location', 'northwest');
grid on;

%% 4. Exercise: Compute Hover Speed
% --------------------------------------------------------------------------
% QUESTION: What propeller speed is required for a quadcopter of mass
% M = 0.54 kg with thrust coefficient kT = 1.072e-5 to hover?
% Use g = 9.81 m/s^2.
%
% PHYSICS: At hover, total thrust = weight.
%   4 * T = M * g          (four propellers share the load equally)
%   4 * kT * omega^2 = M * g
%   omega_hover = sqrt(M * g / (4 * kT))
% --------------------------------------------------------------------------

M_hover  = 0.54;          % quadcopter mass [kg]
g        = 9.81;          % gravitational acceleration [m/s^2]
kT_hover = 1.072e-5;      % thrust coefficient for this propeller

omega_hover = sqrt(M_hover * g / (4 * kT_hover));

fprintf('\n--- Hover Speed Calculation ---\n');
fprintf('Quadcopter mass:      M   = %.3f kg\n', M_hover);
fprintf('Thrust coefficient:   kT  = %.4e N/(rad/s)^2\n', kT_hover);
fprintf('Required omega:       omega_hover = %.2f rad/s\n', omega_hover);
fprintf('Equivalent RPM:       %.0f RPM\n', 60 * omega_hover / (2*pi));
fprintf('Thrust per propeller: %.3f N (should equal Mg/4 = %.3f N)\n', ...
        kT_hover * omega_hover^2, M_hover * g / 4);

%% 5. Local Function: SimplePropellerModel
% --------------------------------------------------------------------------
% Encapsulates the propeller physics as a reusable function.
% Inputs:
%   omega   - rotational speed [rad/s]  (scalar or vector)
%   kT      - thrust coefficient       [N/(rad/s)^2]
%   km      - drag coefficient         [N*m/(rad/s)^2]
%   spinDir - +1 (CCW) or -1 (CW)
% Outputs:
%   T       - thrust force             [N]
%   Q       - reaction torque          [N*m]
% --------------------------------------------------------------------------

function [T, Q] = SimplePropellerModel(omega, kT, km, spinDir)
    T = kT * omega.^2;  % Thrust [N]
    Q = spinDir * km * omega.^2; % Reaction torque [N*m]     
end