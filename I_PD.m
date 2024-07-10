% Parametri regulatora
Kp = 14.88;       % Proporcionalni koeficijent
Ki = 270.5454;     % Integralni koeficijent
Kd = 0.14136;    % Diferencijalni koeficijent
Ts = 0.005;   % Period uzorkovanja
N = 100;     % Broj iteracija simulacije

% Parametri sustava
a1 = -1.689;
a2 = 0.6886;
b0 = 0.0056831388;
b1 = 0.0050134512;

% Inicijalizacija varijabli
y = zeros(1, N);
u = zeros(1, N);
e = zeros(1, N);
ref = zeros(1, N);
 % Referentni signal

% Postavljanje step funkcije kao ulaznog signala
step_time = 10;  % Vrijeme kad se dogodi skok
ref(step_time:end) = 1;

integral = 0;
previous_error = 0;

% Simulacija
for k = 3:N
    % Pogreška
    e(k) = ref(k) - y(k-1);
    
    
    prop = Kp * y(k-1);
    
    % Integracija
    integral = integral + Ki * e(k) * Ts;
    
    % Derivacija
    derivative = (Kd* (y(k-1)-y(k-2)))/ Ts;
    
    % Izlaz PID regulatora
    u(k) =   integral - ( derivative + prop);
    
    % Zasićenje izlaza regulatora
    if u(k) > 5
        u(k) = 5;
        integral = u(k) - prop - derivative;  % Reset integratora
    elseif u(k) < -5
        u(k) = -5;
        integral = u(k) - prop - derivative;  % Reset integratora
    end
    
    % Diskretni model procesa
    y(k) = (b0 * u(k-1) + b1 * u(k-2) - a1 * y(k-1) - a2 * y(k-2));
    
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
