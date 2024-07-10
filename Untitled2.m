% Parametri regulatora
Kp = 14.88;       % Proporcionalni koeficijent
Ki = 1.3527;      % Integralni koeficijent
Kd = 28.272;      % Diferencijalni koeficijent
Ts = 0.005;       % Period uzorkovanja
N = 1000;         % Broj iteracija simulacije

% Parametri sustava
a0 = 1;
a1 = -1.689;
a2 = 0.6886;
b0 = 0.0056831388;
b1 = 0.0050134512;

% Inicijalizacija varijabli
y = zeros(1, N);
u = zeros(1, N);
e = zeros(1, N);
ref = zeros(1, N);  % Referentni signal

% Postavljanje step funkcije kao ulaznog signala
step_time = 200;  % Vrijeme kad se dogodi skok
ref(step_time:end) = 1;

integral = 0;
previous_error = 0;

% Simulacija
for k = 3:N
    % Pogreška
    e(k) = ref(k) - y(k-1);
    
    % Integracija
    integral = integral + e(k) * Ts;
    
    % Derivacija
    derivative = (e(k) - previous_error) / Ts;
    
    % Izlaz PID regulatora
    u(k) = Kp * e(k) + Ki * integral + Kd * derivative;
    
    % Zasićenje izlaza regulatora
    if u(k) > 5
        u(k) = 5;
        integral = integral - e(k) * Ts;  % Reset integratora
    elseif u(k) < -5
        u(k) = -5;
        integral = integral - e(k) * Ts;  % Reset integratora
    end
    
    % Diskretni model procesa
    y(k) = (b0 * u(k) + b1 * u(k-1) - a1 * y(k-1) - a2 * y(k-2)) / a0;
    
    % Spremanje trenutne pogreške za iduću iteraciju
    previous_error = e(k);
end

% Grafički prikaz rezultata
figure;
subplot(2, 1, 1);
plot(1:N, ref, '--', 1:N, y, 'LineWidth', 2);
title('Odziv sustava');
xlabel('Iteracija');
ylabel('Izlaz');
legend('Referenca', 'Izlaz');
grid on;

subplot(2, 1, 2);
plot(1:N, u, 'LineWidth', 2);
title('Izlaz regulatora');
xlabel('Iteracija');
ylabel('u(k)');
grid on;
