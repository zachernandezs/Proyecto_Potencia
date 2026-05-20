clear; clc; close all;

%  SECCIÓN 1 — PARÁMETROS DEL CONVERTIDOR 

Vin  = 12;          % [V]   Tensión de entrada
Vout = 24;          % [V]   Tensión de salida deseada
D    = 0.5;         % [-]   Ciclo de trabajo nominal
L    = 200e-6;      % [H]   Inductancia  
C    = 47e-6;       % [F]   Capacitancia 
R    = 100;         % [Ω]   Resistencia de carga nominal
fsw  = 50e3;        % [Hz]  Frecuencia de conmutación
Ts   = 200e-6;      % [s]   Período de muestreo del lazo de control (5 kHz)


%  SECCIÓN 2 — PUNTO DE OPERACIÓN Y VERIFICACIÓN CCM

fprintf('=== PUNTO DE OPERACIÓN ===\n');
Vout_calc = Vin / (1 - D);
fprintf('  Vout calculado  = %.2f V  (objetivo: %.1f V)\n', Vout_calc, Vout);

Iout  = Vout / R;
IL    = Iout / (1 - D);           % Corriente media del inductor
dIL   = Vin * D / (L * fsw);      % Rizado pico a pico de corriente
IL_min = IL - dIL/2;

fprintf('  Iout = %.4f A\n', Iout);
fprintf('  IL   = %.4f A  (corriente media inductor)\n', IL);
fprintf('  ΔiL  = %.4f A  (rizado p-p)\n', dIL);
fprintf('  iL_min = %.4f A  ', IL_min);
if IL_min > 0
    fprintf(' CCM garantizado \n');
elseif IL_min == 0
    fprintf(' LÍMITE CCM/DCM — revisar L \n');
else
    fprintf(' DCM — AUMENTAR L \n');
end

dVout_pct = (Iout * D / (C * fsw)) / Vout * 100;
fprintf('  Rizado Vout = %.2f%%  (límite: 5%%)\n\n', dVout_pct);


%  SECCIÓN 2.1 — MODELO PROMEDIO Y LINEALIZACIÓN
%  EN ESPACIO DE ESTADOS


fprintf('=== MODELO PROMEDIO LINEALIZADO EN ESPACIO DE ESTADOS ===\n');

% Factor asociado al intervalo de apagado del interruptor
a_op = 1 - D;

% Punto de operación
Vo_op = Vout;
Io_op = Vo_op / R;
IL_op = Io_op / a_op;

fprintf('  Punto de operación usado para linealizar:\n');
fprintf('    Vin = %.2f V\n', Vin);
fprintf('    Vo  = %.2f V\n', Vo_op);
fprintf('    D   = %.4f\n', D);
fprintf('    Io  = %.4f A\n', Io_op);
fprintf('    IL  = %.4f A\n\n', IL_op);

% Matrices del modelo linealizado
% Estado: x_hat = [iL_hat ; vo_hat]
% Entrada: d_hat
% Salida: vo_hat

A_boost = [ 0,        -a_op/L;
            a_op/C,   -1/(R*C) ];

B_boost = [ Vo_op/L;
           -IL_op/C ];

C_boost = [0 1];

D_boost = 0;

fprintf('  Matriz A del modelo linealizado:\n');
disp(A_boost);

fprintf('  Matriz B del modelo linealizado:\n');
disp(B_boost);

fprintf('  Matriz C del modelo linealizado:\n');
disp(C_boost);

fprintf('  Matriz D del modelo linealizado:\n');
disp(D_boost);

% Sistema en espacio de estados
sys_boost_ss = ss(A_boost, B_boost, C_boost, D_boost);

% Función de transferencia obtenida desde espacio de estados
Gvd_ss = minreal(tf(sys_boost_ss));

fprintf('  Función Gvd(s) obtenida desde espacio de estados:\n');
Gvd_ss

%% =========================================================
%  SECCIÓN 3 — MODELO PROMEDIO LINEALIZADO → Gvd(s)

% Parámetros del modelo linealizado
K_plant = Vout / (1 - D);          % Ganancia DC de la planta

% Frecuencia natural y factor de calidad del denominador
w0  = (1 - D) / sqrt(L * C);       % [rad/s] frecuencia natural
Q   = R * (1 - D) * sqrt(C / L);   % factor de calidad

% Cero en RHP (Right Half Plane zero) — limita el ancho de banda
wz  = R * (1 - D)^2 / L;           % [rad/s]
fz  = wz / (2*pi);
f0  = w0 / (2*pi);

fprintf('=== MODELO DE LA PLANTA Gvd(s) ===\n');
fprintf('  Ganancia DC  K = %.4f V\n', K_plant);
fprintf('  Frec. natural f0 = %.2f Hz\n', f0);
fprintf('  Factor calidad Q = %.4f\n', Q);
fprintf('  Cero RHP      fz = %.2f Hz  (limita BW del control)\n\n', fz);

% Construcción de Gvd(s) con MATLAB Control Toolbox
s = tf('s');

num_plant = K_plant * (1 - s/wz);
den_plant = 1 + s/(Q*w0) + (s/w0)^2;
Gvd = num_plant / den_plant;

fprintf('  Función de transferencia Gvd(s):\n');
Gvd

% Comparación entre la planta obtenida desde espacio de estados
% y la forma estándar usada para el diseño del PI
fprintf('  Comparación entre Gvd(s) desde espacio de estados y forma estándar:\n');
Gvd_error = minreal(Gvd - Gvd_ss);
Gvd_error


%  SECCIÓN 4 — ANÁLISIS DE LA PLANTA

figure('Name','Análisis de Planta Gvd(s)','NumberTitle','off');

% Diagrama de Bode
subplot(2,1,1);
bode(Gvd);
title('Diagrama de Bode — Planta Gvd(s)');
grid on;

% Respuesta al escalón de la planta (lazo abierto)
subplot(2,1,2);
step(Gvd * 0.01);   % pequeña perturbación de duty
title('Respuesta al escalón (planta en lazo abierto, Δd = 0.01)');
xlabel('Tiempo [s]'); ylabel('Δvout [V]'); grid on;

% Polos y ceros
figure('Name','Mapa de polos y ceros','NumberTitle','off');
pzmap(Gvd);
title('Mapa de polos y ceros — Gvd(s)');
grid on;
[poles_p, zeros_p] = pzmap(Gvd);
fprintf('=== POLOS Y CEROS DE Gvd(s) ===\n');
fprintf('  Polos: '); disp(poles_p.');
fprintf('  Ceros: '); disp(zeros_p.');


%  SECCIÓN 5 — DISEÑO DEL CONTROLADOR PI

%  - Objetivos:
%      a) margen de fase ≥ 45° (preferible 60°)
%      b) fc << fz (cero RHP limita el ancho de banda alcanzable)
%      c) error estático = 0 (garantizado por la integral)


PM_deseado = 45;    % [°] margen de fase mínimo aceptable

% Rango de búsqueda: desde 10 Hz hasta fz/8
fc_min_search = 10;
fc_max_search = fz / 8;
fc_candidates = logspace(log10(fc_min_search), log10(fc_max_search), 300);

fprintf('=== DISEÑO DEL CONTROLADOR PI ===\n');
fprintf('  Buscando fc óptimo entre %.1f Hz y %.1f Hz\n', fc_min_search, fc_max_search);
fprintf('  (RHP zero fz = %.1f Hz  límite fc < fz/8 = %.1f Hz)\n\n', fz, fz/8);

% Barrido: calcular PM para cada fc candidato
PM_vec  = zeros(size(fc_candidates));
Kp_vec  = zeros(size(fc_candidates));
Ti_vec  = zeros(size(fc_candidates));

for ii = 1:length(fc_candidates)
    fc_try = fc_candidates(ii);
    wc_try = 2*pi*fc_try;

    % Ti: cero del PI a fc/10 (una década abajo → mínima degradación de fase)
    Ti_try = 10 / wc_try;

    % PI con Kp=1 para evaluar magnitud
    C_try = (1 + 1/(Ti_try * s));

    % Magnitud del lazo L = C_try * Gvd en wc_try
    mag_L = squeeze(abs(freqresp(C_try * Gvd, wc_try)));

    % Kp para que |L(jwc)| = 1
    Kp_try = 1 / mag_L;

    % Lazo abierto real con este Kp
    L_try = Kp_try * C_try * Gvd;

    % Margen de fase real 
    [~, PM_try] = margin(L_try);

    PM_vec(ii)  = PM_try;
    Kp_vec(ii)  = Kp_try;
    Ti_vec(ii)  = Ti_try;
end

% Filtrar candidatos con PM >= PM_deseado y elegir el de mayor fc
% (mayor fc = respuesta más rápida)
idx_valid = find(PM_vec >= PM_deseado);

if isempty(idx_valid)
    % Si ninguno alcanza 45°, elegir el de mayor PM disponible
    [~, idx_best] = max(PM_vec);
    fprintf('   No se encontró fc con PM ≥ %d°. Usando mejor disponible.\n', PM_deseado);
else
    idx_best = idx_valid(end);   % mayor fc con PM aceptable
end

fc = fc_candidates(idx_best);
wc = 2*pi*fc;
Ti = Ti_vec(idx_best);
Kp = Kp_vec(idx_best);
Ki = Kp / Ti;

fprintf('  fc seleccionado  = %.2f Hz\n', fc);
fprintf('  PM obtenido      = %.2f°\n\n', PM_vec(idx_best));
fprintf('  PARÁMETROS DEL CONTROLADOR PI:\n');
fprintf('    Kp = %.6f\n', Kp);
fprintf('    Ti = %.6f s\n', Ti);
fprintf('    Ki = Kp/Ti = %.6f\n', Ki);
fprintf('    Cero del PI en: 1/(2*pi*Ti) = %.4f Hz\n\n', 1/(2*pi*Ti));

% Gráfica del barrido PM vs fc 
figure('Name','Barrido PM vs fc','NumberTitle','off');
semilogx(fc_candidates, PM_vec, 'b-', 'LineWidth', 1.5); hold on;
yline(PM_deseado, 'r--', 'LineWidth', 1);
xline(fc, 'g--', 'LineWidth', 1);
scatter(fc, PM_vec(idx_best), 60, 'g', 'filled');
xlabel('Frecuencia de cruce fc [Hz]');
ylabel('Margen de fase PM [°]');
title('Margen de fase alcanzable vs fc — PI con cero en fc/10');
legend('PM(fc)', sprintf('PM objetivo %d°', PM_deseado), ...
       sprintf('fc elegido = %.1f Hz', fc), 'Location','SouthWest');
grid on;

% Controlador PI en tiempo continuo
C_PI = Kp * (1 + 1/(Ti * s));
fprintf('  Controlador PI C(s):\n');
C_PI


%  SECCIÓN 6 — ANÁLISIS DEL LAZO CERRADO

% Función de lazo abierto (sin retroalimentación unitaria)
L_open = C_PI * Gvd;

% Márgenes de estabilidad
[Gm, Pm, Wcg, Wcp] = margin(L_open);
fprintf('=== ANÁLISIS DEL LAZO CERRADO ===\n');
fprintf('  Margen de ganancia  = %.2f dB  (en %.2f Hz)\n', 20*log10(Gm), Wcg/(2*pi));
fprintf('  Margen de fase      = %.2f°    (en %.2f Hz)\n', Pm, Wcp/(2*pi));
if Pm >= 45
    fprintf('   Estabilidad: ACEPTABLE \n\n');
elseif Pm >= 30
    fprintf('   Estabilidad: MARGINAL  (considerar aumentar PM)\n\n');
else
    fprintf('   Estabilidad: INSUFICIENTE  — rediseñar\n\n');
end

% Función de transferencia en lazo cerrado
T_closed = feedback(L_open, 1);

figure('Name','Lazo abierto y cerrado','NumberTitle','off');
subplot(2,1,1);
margin(L_open);
title('Diagrama de Bode — Lazo abierto C(s)·Gvd(s)');
grid on;

subplot(2,1,2);
step(T_closed * Vout);
title(sprintf('Respuesta al escalón — Lazo cerrado (Ref = %.0f V)', Vout));
xlabel('Tiempo [s]'); ylabel('Vout [V]'); grid on;

% Información de la respuesta
info = stepinfo(T_closed * Vout);
fprintf('  Tiempo de asentamiento (2%%)  = %.4f s\n', info.SettlingTime);
fprintf('  Sobreimpulso               = %.2f%%\n', info.Overshoot);
fprintf('  Tiempo de subida           = %.4f s\n\n', info.RiseTime);


%  SECCIÓN 7 — DISCRETIZACIÓN DEL PI (Método Tustin / Bilineal)


% Discretización usando c2d con Tustin
C_PI_d = c2d(C_PI, Ts, 'tustin');

fprintf('=== DISCRETIZACIÓN DEL PI (Tustin, Ts = %.0f µs) ===\n', Ts*1e6);
fprintf('  Controlador discreto C(z):\n');
C_PI_d

% Coeficientes para implementar en ensamblador (ecuación de diferencias)
[num_d, den_d] = tfdata(C_PI_d, 'v');
b0 = num_d(1);
b1 = num_d(2);
% den_d = [1, -1] siempre para PI con Tustin

% Fórmula explícita 
b0_formula = Kp + Ki*Ts/2;
b1_formula = -Kp + Ki*Ts/2;

fprintf('\n  ECUACIÓN DE DIFERENCIAS:\n');
fprintf('  u[k] = u[k-1] + b0*e[k] + b1*e[k-1]\n\n');
fprintf('  b0 = Kp + Ki*Ts/2 = %.6f\n', b0_formula);
fprintf('  b1 = -Kp + Ki*Ts/2 = %.6f\n\n', b1_formula);
fprintf('  Verificación con c2d:\n');
fprintf('  b0 = %.6f  b1 = %.6f\n\n', b0, b1);

% Escalar a representación de punto fijo 
fprintf('  REPRESENTACIÓN EN PUNTO FIJO (escala Q16):\n');
scale = 2^16;
fprintf('  b0_int = %d\n', round(b0_formula * scale));
fprintf('  b1_int = %d\n\n', round(b1_formula * scale));


%  SECCIÓN 8 — SIMULACIÓN DISCRETA DEL LAZO CERRADO

% Planta discreta (muestreada con ZOH para la simulación)
Gvd_d = c2d(Gvd, Ts, 'zoh');

% Lazo cerrado discreto
T_closed_d = feedback(C_PI_d * Gvd_d, 1);

%  Simulación 1: respuesta al escalón de referencia ---
t_sim  = 0 : Ts : 0.2;          
ref    = Vout * ones(size(t_sim));  % referencia 24 V
[y_step, t_step] = step(T_closed_d * Vout, t_sim);

figure('Name','Simulación discreta del lazo cerrado','NumberTitle','off');
subplot(3,1,1);
plot(t_step*1000, y_step, 'b', 'LineWidth', 1.5); hold on;
plot(t_step*1000, ref, 'r--', 'LineWidth', 1);
plot(t_step*1000, Vout*1.05*ones(size(t_step)), 'k:', 'LineWidth', 0.8);
plot(t_step*1000, Vout*0.95*ones(size(t_step)), 'k:', 'LineWidth', 0.8);
xlabel('Tiempo [ms]'); ylabel('Vout [V]');
title('Respuesta al escalón — PI discreto (Ts = 200 µs)');
legend('Vout', 'Referencia 24V', '±5% banda', 'Location', 'SouthEast');
grid on;

%  Simulación 2: cambio de referencia (de 20V a 24V) 
t_sim2 = 0 : Ts : 0.4;           
u_ref2 = [20*ones(1, round(0.01/Ts)), 24*ones(1, length(t_sim2)-round(0.01/Ts))];
y_sim2 = lsim(T_closed_d, u_ref2, t_sim2);

subplot(3,1,2);
plot(t_sim2*1000, y_sim2, 'b', 'LineWidth', 1.5); hold on;
plot(t_sim2*1000, u_ref2, 'r--', 'LineWidth', 1);
xlabel('Tiempo [ms]'); ylabel('Vout [V]');
title('Cambio de referencia: 20V a 24V en t = 10 ms');
legend('Vout', 'Referencia', 'Location', 'SouthEast');
grid on;

%  Simulación 3: perturbación de carga (R cambia de 100Ω a 50Ω) 
% Modelar el cambio de carga como perturbación aditiva en la salida

S_closed = feedback(1, C_PI * Gvd);   % función de sensibilidad
S_closed_d = c2d(S_closed, Ts, 'tustin');

% Perturbación: cambio de carga equivale a corriente adicional
% ΔI = Vout/R_new - Vout/R_old = 24/50 - 24/100 = 0.24 A
delta_I = Vout/50 - Vout/100;   % 0.24 A de perturbación de corriente

t_sim3 = 0:Ts:0.4;     
pert = [zeros(1,round(0.01/Ts)), delta_I*ones(1,length(t_sim3)-round(0.01/Ts))];
y_pert = lsim(S_closed_d * (-R), pert, t_sim3);  % respuesta a perturbación

subplot(3,1,3);
plot(t_sim3*1000, Vout + y_pert, 'b', 'LineWidth', 1.5); hold on;
yline(Vout, 'r--', 'LineWidth', 1);
yline(Vout*1.05, 'k:', 'LineWidth', 0.8);
yline(Vout*0.95, 'k:', 'LineWidth', 0.8);
xlabel('Tiempo [ms]'); ylabel('Vout [V]');
title(sprintf('Respuesta a cambio de carga: 100Ω a 50Ω en t = 10ms (ΔI = %.2f A)', delta_I));
legend('Vout', 'Referencia 24V', '±5% banda', 'Location', 'SouthEast');
grid on;

sgtitle('Simulación lazo cerrado con PI discreto — Convertidor Boost 12 a 24V');


%  SECCIÓN 9 — Resumen de resultados

fprintf('=== RESUMEN ===\n');
fprintf('--------------------------------------------------\n');
fprintf('  Parámetros del convertidor\n');
fprintf('    Vin = %.0f V, Vout = %.0f V, D = %.2f\n', Vin, Vout, D);
fprintf('    L = %.0f µH, C = %.2f µF, R = %.0f Ω\n', L*1e6, C*1e6, R);
fprintf('    fsw = %.0f kHz, Ts_control = %.0f µs\n', fsw/1e3, Ts*1e6);
fprintf('  Planta Gvd(s)\n');
fprintf('    Ganancia DC = %.4f V\n', K_plant);
fprintf('    f0 = %.2f Hz, Q = %.4f\n', f0, Q);
fprintf('    Cero RHP fz = %.2f Hz\n', fz);
fprintf('  Controlador PI (tiempo continuo)\n');
fprintf('    Kp = %.6f\n', Kp);
fprintf('    Ki = %.6f\n', Ki);
fprintf('    Cero PI en = %.4f Hz\n', 1/(2*pi*Ti));
fprintf('  Controlador PI (discreto, Tustin)\n');
fprintf('    b0 = %.6f\n', b0_formula);
fprintf('    b1 = %.6f\n', b1_formula);
fprintf('    Ecuación: u[k] = u[k-1] + b0*e[k] + b1*e[k-1]\n');
fprintf('  Márgenes de estabilidad (lazo continuo)\n');
fprintf('    Margen de ganancia = %.2f dB\n', 20*log10(Gm));
fprintf('    Margen de fase     = %.2f°\n', Pm);
fprintf('--------------------------------------------------\n');
fprintf('\nScript completado exitosamente.\n');