%% Constantes fisicas

J  = 4.0771e-08;    % Momento de inercia           [kg·m^2]
b  = 4.7047e-08;    % Fricción viscosa             [N·m·s/rad]
Ke = 0.00335915;    % Cte. fuerza contraelectromotriz [V·s/rad]
Kt = 0.00335915;    % Cte. de par                  [N·m/A]
R  = 10.3;          % Resistencia de armadura      [Ohm]
L  = 0.0711e-3;     % Inductancia de armadura      [H] (0.0711 mH)


%% Parámetros derivados

tau_m = J / (b + Kt*Ke/R);     % Constante de tiempo mecánica [s]
K     = Kt / (R*b + Kt*Ke);    % Ganancia estática [(rad/s)/V]


%% Parámetros de simulación

x0 = 0;                        % Motor parado [rad/s]
u  = 5;                        % Tensión constante [V]
s  = 10;                       % Muestras por tau_m
Ts = tau_m / s;                % Periodo de muestreo [s]
N  = ceil(0.5 / Ts);           % Sumalar durante 0.5s

xss = K*u;                     % Regimen permanente

assert(Ts <= tau_m, 'Ts debe ser <= tau_m (estabilidad y sin oscilaciones)');

%% Simulacion

[t_vec, x_vec] = simulate_euler(x0, u, tau_m, K, Ts, N);

w_exacta = K*u*(1 - exp(-t_vec/tau_m)) + x0*exp(-t_vec/tau_m);

figure;
plot(t_vec, x_vec, 'o-', 'DisplayName', sprintf('Euler (T_s = %.2f ms)', Ts*1e3));
hold on;
plot(t_vec, w_exacta, '-', 'LineWidth', 1.5, 'DisplayName', 'Exacta (continuo)');
yline(xss, '--', 'DisplayName', 'regimen permanente');
xlabel('t [s]');
ylabel('\omega [rad/s]');
title(sprintf('Respuesta al escalón de %g V', u));
legend('Location', 'southeast');
grid on;

%% Funciones

function [t_vec, x_vec] = simulate_euler(x0, u, tau_m, K, Ts, N)
k_vec = 0:N;
t_vec = k_vec * Ts;

x_vec = zeros(size(t_vec));
x_vec(1) = x0;

for k = 1:N
    x_vec(k + 1) = (1 - Ts/tau_m) * x_vec(k) + (Ts*K/tau_m) * u;
end
end
