% integrated_thermostat_system.m
% This script integrates five subsystem models for a smart thermostat:
%   - Mechanical (actuator dynamics)
%   - Electrical (sensor signal conditioning)
%   - Fluidic (airflow dynamics)
%   - Thermal (indoor temperature dynamics)
%   - Chemical (indoor CO2 concentration dynamics)
%
% Save this script as integrated_thermostat_system.m and run it in MATLAB.

clear; clc; close all;

%% Simulation Time and Initial Conditions
tspan = [0, 3600];   % Simulation time: 0 to 3600 seconds (1 hour)

% Initial conditions for each subsystem:
x0 = [0; 0];      % Actuator: [displacement; velocity]
V0 = 0;           % Sensor: initial voltage
Q0 = 0;           % Airflow: initial airflow rate (m^3/s)
T0 = 20;          % Indoor temperature (°C)
C0 = 400;         % CO2 concentration (ppm)

%% Define Input Functions for Each Subsystem

% Mechanical: Actuator control input (e.g., a sinusoidal control force)
actuator_input = @(t) 1.0 * sin(0.01*t);

% Electrical: Indoor temperature profile for sensor input (°C)
temperature_input = @(t) 20 + 2*sin(0.005*t);

% Fluidic: Fan pressure input for airflow (arbitrary units)
fanPressureInput = @(t) 0.5 + 0.1*cos(0.01*t);

% Thermal: Heating input function (W), active for the first half hour
Q_in_func = @(t) 500 * (t < 1800);

% Chemical: CO2 generation rate (ppm/s) during occupancy (from 10 min to 50 min)
co2_input = @(t) 10 * (t >= 600 & t <= 3000);

%% Run Each Subsystem Model

[t_act, x] = actuator_model(actuator_input, tspan, x0);
[t_sens, V_out] = sensor_control_model(temperature_input, tspan, V0);
[t_air, Q] = airflow_model(fanPressureInput, tspan, Q0);
[t_heat, T] = heat_transfer_model(Q_in_func, tspan, T0);
[t_co2, C] = air_quality_model(co2_input, tspan, C0);

%% Plot Integrated Results

figure;

subplot(3,2,1);
plot(t_act, x(:,1), 'LineWidth', 2);
title('Actuator Displacement');
xlabel('Time (s)');
ylabel('Displacement (m)');

subplot(3,2,2);
plot(t_act, x(:,2), 'LineWidth', 2);
title('Actuator Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');

subplot(3,2,3);
plot(t_sens, V_out, 'LineWidth', 2);
title('Sensor Voltage Output');
xlabel('Time (s)');
ylabel('Voltage (V)');

subplot(3,2,4);
plot(t_air, Q, 'LineWidth', 2);
title('Airflow Rate');
xlabel('Time (s)');
ylabel('Flow (m^3/s)');

subplot(3,2,5);
plot(t_heat, T, 'LineWidth', 2);
title('Indoor Temperature');
xlabel('Time (s)');
ylabel('Temperature (°C)');

subplot(3,2,6);
plot(t_co2, C, 'LineWidth', 2);
title('CO2 Concentration');
xlabel('Time (s)');
ylabel('CO2 (ppm)');

sgtitle('Integrated Smart Thermostat System Simulation');

%% --- Local Function Definitions ---
% The following functions define the dynamics of each subsystem.

function [t, x] = actuator_model(u, tspan, x0)
    % actuator_model simulates the dynamics of an actuator (e.g., damper/fan)
    % using a second-order mass-spring-damper model.
    %
    % Inputs:
    %   u     : function handle for control input (e.g., @(t) sin(t))
    %   tspan : simulation time span, e.g., [0 3600]
    %   x0    : initial state [displacement; velocity]
    %
    % Outputs:
    %   t     : time vector from ODE solver
    %   x     : state matrix [displacement, velocity]
    
    m = 0.3;      % mass in kg
    c = 0.05;     % damping coefficient in Ns/m
    k = 150;      % spring constant in N/m
    
    ode_fun = @(t, x) [x(2); (u(t) - c*x(2) - k*x(1)) / m];
    [t, x] = ode45(ode_fun, tspan, x0);
end

function [t, V_out] = sensor_control_model(T_input, tspan, V0)
    % sensor_control_model simulates a sensor's signal conditioning as a
    % first-order low-pass filter converting temperature (°C) to voltage (V).
    %
    % Inputs:
    %   T_input : function handle for temperature input, e.g., @(t) 20+2*sin(0.005*t)
    %   tspan   : simulation time span, e.g., [0 3600]
    %   V0      : initial sensor voltage (V)
    %
    % Outputs:
    %   t       : time vector from ODE solver
    %   V_out   : sensor voltage output over time
    
    tau  = 1;       % time constant (s)
    gain = 0.1;     % gain (V/°C)
    
    ode_fun = @(t, V) (-1/tau * V + gain * T_input(t));
    [t, V_out] = ode45(ode_fun, tspan, V0);
end

function [t, Q] = airflow_model(u, tspan, Q0)
    % airflow_model simulates the airflow dynamics in an HVAC duct system
    % using a first-order model.
    %
    % Inputs:
    %   u     : function handle for fan pressure input (e.g., @(t) 0.5+0.1*cos(0.01*t))
    %   tspan : simulation time span, e.g., [0 3600]
    %   Q0    : initial airflow rate (m^3/s)
    %
    % Outputs:
    %   t     : time vector from ODE solver
    %   Q     : airflow rate over time (m^3/s)
    
    tau_air = 0.5;  % time constant (s)
    ode_fun = @(t, Q) (-Q + u(t)) / tau_air;
    [t, Q] = ode45(ode_fun, tspan, Q0);
end

function [t, T] = heat_transfer_model(Q_in, tspan, T0)
    % heat_transfer_model simulates indoor temperature dynamics using a lumped
    % thermal model based on an energy balance.
    %
    % Inputs:
    %   Q_in  : function handle for heating/cooling input (W)
    %   tspan : simulation time span, e.g., [0 3600]
    %   T0    : initial indoor temperature (°C)
    %
    % Outputs:
    %   t     : time vector from ODE solver
    %   T     : indoor temperature over time (°C)
    
    C = 3000;         % thermal capacity (J/°C)
    U = 10;           % overall heat loss coefficient (W/°C)
    T_ambient = 15;   % ambient temperature (°C)
    
    ode_fun = @(t, T) (Q_in(t) - U*(T - T_ambient)) / C;
    [t, T] = ode45(ode_fun, tspan, T0);
end

function [t, C_out] = air_quality_model(u, tspan, C0)
    % air_quality_model simulates indoor CO2 concentration dynamics using a
    % first-order model that considers pollutant generation and removal.
    %
    % Inputs:
    %   u     : function handle for CO2 generation rate (ppm/s)
    %   tspan : simulation time span, e.g., [0 3600]
    %   C0    : initial CO2 concentration (ppm)
    %
    % Outputs:
    %   t     : time vector from ODE solver
    %   C_out : CO2 concentration over time (ppm)
    
    tau_CO2  = 300;  % time constant for ventilation (s)
    C_ambient = 400; % ambient CO2 concentration (ppm)
    
    ode_fun = @(t, C_out) (u(t) - (C_out - C_ambient)) / tau_CO2;
    [t, C_out] = ode45(ode_fun, tspan, C0);
end
