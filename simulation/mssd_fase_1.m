%% Constantes de medidas

ENCODER_PPR = 24;              % Pulsos por vuelta (CHANGE en canal A)

%% Constantes fisicas

J  = 0.01244;    % Momento de inercia           [kg·m^2]
b  = 4.7047e-08;    % Fricción viscosa             [N·m·s/rad]
Ke = 0.01879;    % Cte. fuerza contraelectromotriz [V·s/rad]
Kt = 0.01879;    % Cte. de par                  [N·m/A]
R  = 6;          % Resistencia de armadura      [Ohm]
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

%% Datos reales (firmware/capturar)

csv_path = fullfile(fileparts(mfilename('fullpath')), '..', 'data', 'fase1.csv');
datos = readmatrix(csv_path);
t_real = datos(:, 1) * 1e-6;   % [s]
pulsos = datos(:, 2);

% k_w = (W + 1):numel(t_real);
k_w = 2:numel(t_real);
w_real = 2*pi/ENCODER_PPR * (pulsos(k_w) - pulsos(k_w - 1)) ./ (t_real(k_w) - t_real(k_w - 1));
t_w = t_real(k_w);

% Constante de tiempo real: primer instante con 63.2 % de la velocidad de establecimiento
w_ss_real = mean(w_real(end - floor(numel(w_real)/5):end));   % último 20 % del ensayo
tau_real = t_w(find(w_real >= 0.632*w_ss_real, 1));
fprintf('tau_m real (63.2 %%)    = %.2f ms\n', tau_real*1e3);

%% Simulacion

[t_vec, x_vec] = simulate_euler(x0, u, tau_m, K, Ts, N);

w_exacta = K*u*(1 - exp(-t_vec/tau_m)) + x0*exp(-t_vec/tau_m);

figure;
% plot(t_vec, x_vec, 'o-', 'DisplayName', sprintf('Euler (T_s = %.2f ms)', Ts*1e3));
hold on;
% plot(t_vec, w_exacta, '-', 'LineWidth', 1.5, 'DisplayName', 'Exacta (continuo)');
plot(t_w, w_real, '-', 'LineWidth', 1.5, 'DisplayName', 'Real (encoder)');
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
