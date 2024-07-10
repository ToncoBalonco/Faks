% Define system parameters for I-PD controller
KR = 14.88;
TI = 0.055;
TD = 0.0095;

T = 0.005; % Discretization time

% Define the discrete transfer function G(z)
numG = [0.005683, 0.0050134512];
denG = [1, -1.6886, 0.6886];

% Define the I-PD controller parameters
a0 = KR * (1 + T / (2 * TI) + TD / T);
a1 = KR * (-1 + T / (2 * TI) - 2 * TD / T);
a2 = KR * TD / T;

% Initialize variables for I-PD controller
u_prev = 0; % Previous control signal
e_prev1 = 0; % Error at previous step
e_prev2 = 0; % Error at two steps back
integrator = 0; % Integrator state

% Saturation limits for control signal
u_min = -5;
u_max = 5;

% Initial conditions for the process
y = 0;
y_prev1 = 0;
y_prev2 = 0;

% Simulation parameters
simulation_time = 5; % seconds
num_steps = simulation_time / T;
y_target = 1; % Desired output (step function amplitude)

% Pre-allocate arrays for storing results
y_array = zeros(num_steps, 1);
u_array = zeros(num_steps, 1);
e_array = zeros(num_steps, 1);
time_array = (0:num_steps-1) * T;

for k = 1:num_steps
    % Calculate the current error
    e = y_target - y;
    
    % I-PD controller difference equation
    u = u_prev + a0 * y + a1 * e_prev1 + a2 * y;
    
    % Apply saturation limits
    if u > u_max
        u = u_max;
        % Anti-windup: reset integrator state
        integrator = 0;
    elseif u < u_min
        u = u_min;
        % Anti-windup: reset integrator state
        integrator = 0;
    end
    
    % Update previous control signal and errors
    u_prev = u;
    e_prev2 = e_prev1;
    e_prev1 = e;
    
    % Process model difference equation (discrete)
    y = -denG(2) * y_prev1 - denG(3) * y_prev2 + numG(1) * u + numG(2) * u_prev;
    
    % Store results
    y_array(k) = y;
    u_array(k) = u;
    e_array(k) = e;
    
    % Update previous process outputs
    y_prev2 = y_prev1;
    y_prev1 = y;
end

% Plot results
figure;
subplot(3, 1, 1);
plot(time_array, y_array);
title('Process Output with I-PD Controller');
xlabel('Time (s)');
ylabel('Output');
grid on;

subplot(3, 1, 2);
plot(time_array, u_array);
title('Control Signal with I-PD Controller');
xlabel('Time (s)');
ylabel('Control Signal (V)');
grid on;

subplot(3, 1, 3);
plot(time_array, e_array);
title('Error Signal with I-PD Controller');
xlabel('Time (s)');
ylabel('Error');
grid on;
