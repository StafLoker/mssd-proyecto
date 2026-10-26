%% Medidas de motor medidas

encoder_ppr = 24; % Pulsos por vuelta (CHANGE en canal A)

R = 6; % Resistencia de armadura      [Ohm]

v_ab = 4.93; % Tension cto abierto [V]
v_c = 3.53; % Tension cto cerrado [V]

i = 0.5e-3; % Corriente estable [mA]
i_stall = 0.5;  % Corriente stall [A]
i_arr = 0.1e-3;  % Corriente de arranque [mA]

w_est = 260.752; % Velocidad angular una vez estabilidado [rad/s]

tau_real = 0.075 ; % Instante donde w es 63.2 % [s]

%% Constantes fisicas

L  = 0.0711e-3;     % Inductancia de armadura      [H] (0.0711 mH)

Ke = (v_ab - i * R) / w_est;    % Cte. fuerza contraelectromotriz [V·s/rad]
Kt = Ke;    % Cte. de par                  [N·m/A]

J  = (tau_real * Ke * Kt / R);    % Momento de inercia           [kg·m^2]
b  = (Kt * i) / w_est;    % Fricción viscosa             [N·m·s/rad]


%% Parámetros derivados

tau_m = J / (b + Kt*Ke/R);     % Constante de tiempo mecánica [s]
K     = Kt / (R*b + Kt*Ke);    % Ganancia estática [(rad/s)/V]


%% Parámetros de simulación

x0 = 0;                        % Motor parado [rad/s]
u  = 5;                        % Tensión constante [V]
s  = 10;                       % Muestras por tau_m
Ts = tau_m / s;                % Periodo de muestreo [s]
N  = ceil(10 / Ts);           % Sumalar durante 10s

assert(Ts <= tau_m, 'Ts debe ser <= tau_m (estabilidad y sin oscilaciones)');

%% Datos reales (firmware/capturar)

% Leer datos

csv_path = fullfile(fileparts(mfilename('fullpath')), '..', 'data', 'fase2.csv');
datos = readmatrix(csv_path);

t_real    = datos(:, 1) * 1e-6;   % [s]
pulsos    = datos(:, 2);
direccion = datos(:, 3);          % 0 = FORWARD, 1 = INVERSE

k_w = 2:numel(t_real);
w_real = 2*pi/encoder_ppr * (pulsos(k_w) - pulsos(k_w - 1)) ./ (t_real(k_w) - t_real(k_w - 1));
t_w = t_real(k_w);

w_real = w_real .* (1 - 2*direccion(k_w)); % INVERSE direccion se toma valores negativos de w

% Calcular pulsos

k_pulso = find(diff(direccion) ~= 0) + 1;
t_pulso = t_real(k_pulso);

%% Simulacion

% [t_vec, x_vec] = simulate_euler(x0, u, tau_m, K, Ts, N);

%% RMSE simulacion vs real

% La simulacion se interpola en los instantes del encoder, dentro de su rango
% en_rango = t_w >= t_vec(1) & t_w <= t_vec(end); % vector bool donde T_sim coinicide con T_real
% w_sim = interp1(t_vec, x_vec, t_w(en_rango));
% rmse = sqrt(mean((w_sim - w_real(en_rango)).^2));

%% Dibujar resultado

figure;
% Prioridad visual: real > Euler
% plot(t_vec, x_vec, 'o-', 'LineWidth', 1.5, 'MarkerSize', 5, 'Color', [0.85 0.33 0.10], 'DisplayName', sprintf('Simulacion Euler (T_s = %.2f ms)', Ts*1e3));
hold on;
yline(0, '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 1, 'HandleVisibility', 'off');
xline(t_pulso, ':', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5, 'HandleVisibility', 'off');
plot(NaN, NaN, ':', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5, 'DisplayName', 'Pulsacion del boton');
plot(t_w, w_real, '-', 'LineWidth', 2.5, 'Color', [0 0.447 0.741], 'DisplayName', 'Real (encoder)');
% text(0.98, 0.35, sprintf('RMSE = %.2f rad/s (%.1f %% de xss)', rmse, 100*rmse/xss), ...
%     'Units', 'normalized', 'HorizontalAlignment', 'right', 'BackgroundColor', 'w', 'EdgeColor', 'k');
xlabel('t [s]');
ylabel('\omega [rad/s]');
title('Velocidad angular con inversión de giro por botón');
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
