clear; clc; close all;

% =========================================================================
% ÚLOHA 2: Vykreslenie trajektórie štvorec
% =========================================================================
disp('--- Úloha 2: Štvorec ---');

% --- Používateľský vstup ---
S_strana = input('Zadajte dĺžku strany štvorca S [m]: ');
if isempty(S_strana) || ~isnumeric(S_strana) || S_strana <= 0
    S_strana = 0.5; % Predvolená hodnota, ak je vstup neplatný
    disp(['Neplatný vstup, použije sa predvolená dĺžka strany: ', num2str(S_strana), ' m']);
end

% --- Konštanty pre pohyb v Úlohe 2 ---
v_lin_task2 = 0.25;      % Lineárna rýchlosť pre pohyb rovno [m/s]
omega_rot_task2 = pi/2; % Uhlová rýchlosť pre otáčanie [rad/s] (90 stupňov za 1s)
L = 0.2; % Rozchod kolies [m]

num_segments_per_side = 1;
num_segments_per_turn = 20;

% --- Generovanie časov a rýchlostí pre štvorec ---
t_current_task2 = 0;
t_input_task2 = [0];
RLKI_task2 = [];
RPKI_task2 = [];

for i_square_leg = 1:4
    duration_straight_total = S_strana / v_lin_task2;
    dt_straight_segment = duration_straight_total / num_segments_per_side;
    vL_val_straight = v_lin_task2;
    vR_val_straight = v_lin_task2;
    for seg = 1:num_segments_per_side
        RLKI_task2 = [RLKI_task2, vL_val_straight];
        RPKI_task2 = [RPKI_task2, vR_val_straight];
        t_current_task2 = t_current_task2 + dt_straight_segment;
        t_input_task2 = [t_input_task2, t_current_task2];
    end

    target_angle_turn = pi/2;
    duration_turn_total = target_angle_turn / omega_rot_task2;
    dt_turn_segment = duration_turn_total / num_segments_per_turn;
    vR_val_turn = omega_rot_task2 * L / 2;
    vL_val_turn = -vR_val_turn;
    for seg = 1:num_segments_per_turn
        RLKI_task2 = [RLKI_task2, vL_val_turn];
        RPKI_task2 = [RPKI_task2, vR_val_turn];
        t_current_task2 = t_current_task2 + dt_turn_segment;
        t_input_task2 = [t_input_task2, t_current_task2];
    end
end

% --- Inicializácia pre simuláciu Úlohy 2 ---
Pocet_intervalov_task2 = length(RLKI_task2);
Pocet_v_Poli_task2 = length(t_input_task2);
Hist_x_task2 = zeros(1, Pocet_v_Poli_task2);
Hist_y_task2 = zeros(1, Pocet_v_Poli_task2);
Hist_fi_task2 = zeros(1, Pocet_v_Poli_task2);
x_L_hist_task2 = zeros(1, Pocet_v_Poli_task2);
y_L_hist_task2 = zeros(1, Pocet_v_Poli_task2);
x_R_hist_task2 = zeros(1, Pocet_v_Poli_task2);
y_R_hist_task2 = zeros(1, Pocet_v_Poli_task2);
v_T_hist_task2 = zeros(1, Pocet_intervalov_task2);
Hist_x_task2(1) = 0; Hist_y_task2(1) = 0; Hist_fi_task2(1) = 0;
x_L_hist_task2(1) = Hist_x_task2(1)-(L/2)*sin(Hist_fi_task2(1)); y_L_hist_task2(1) = Hist_y_task2(1)+(L/2)*cos(Hist_fi_task2(1));
x_R_hist_task2(1) = Hist_x_task2(1)+(L/2)*sin(Hist_fi_task2(1)); y_R_hist_task2(1) = Hist_y_task2(1)-(L/2)*cos(Hist_fi_task2(1));

% --- Simulácia pohybu pre Úlohu 2 ---
if Pocet_intervalov_task2 > 0
    for k = 1:Pocet_intervalov_task2
        dt_task2 = t_input_task2(k+1) - t_input_task2(k);
        v_L_task2 = RLKI_task2(k); v_R_task2 = RPKI_task2(k);
        v_T_val_task2 = (v_R_task2+v_L_task2)/2; omega_val_task2 = (v_R_task2-v_L_task2)/L;
        v_T_hist_task2(k) = v_T_val_task2;
        x_prev_task2 = Hist_x_task2(k); y_prev_task2 = Hist_y_task2(k); phi_prev_task2 = Hist_fi_task2(k);
        Hist_x_task2(k+1) = x_prev_task2+v_T_val_task2*cos(phi_prev_task2)*dt_task2;
        Hist_y_task2(k+1) = y_prev_task2+v_T_val_task2*sin(phi_prev_task2)*dt_task2;
        Hist_fi_task2(k+1) = phi_prev_task2+omega_val_task2*dt_task2;
        Hist_fi_task2(k+1) = mod(Hist_fi_task2(k+1)+pi,2*pi)-pi;
        phi_current_task2 = Hist_fi_task2(k+1); x_T_current_task2 = Hist_x_task2(k+1); y_T_current_task2 = Hist_y_task2(k+1);
        x_L_hist_task2(k+1) = x_T_current_task2-(L/2)*sin(phi_current_task2); y_L_hist_task2(k+1) = y_T_current_task2+(L/2)*cos(phi_current_task2);
        x_R_hist_task2(k+1) = x_T_current_task2+(L/2)*sin(phi_current_task2); y_R_hist_task2(k+1) = y_T_current_task2-(L/2)*cos(phi_current_task2);
    end
end

% --- Vykreslenie priebehov rýchlostí pre Úlohu 2 ---
figure('Name', 'Úloha 2: Priebehy rýchlostí (Štvorec)');

% Prvý podgraf: Rýchlosti kolies
subplot(2,1,1);
hold on;
if Pocet_intervalov_task2 > 0
    stairs(t_input_task2, [RLKI_task2, RLKI_task2(end)],'r-', 'LineWidth', 1.5, 'DisplayName', 'Rýchlosť ľavého kolesa');
    plot(t_input_task2(1:Pocet_intervalov_task2), RLKI_task2, 'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r', 'HandleVisibility','off');

    stairs(t_input_task2, [RPKI_task2, RPKI_task2(end)],'b--', 'LineWidth', 1.5, 'DisplayName', 'Rýchlosť pravého kolesa');
    plot(t_input_task2(1:Pocet_intervalov_task2), RPKI_task2, 'bo', 'MarkerSize', 3, 'MarkerFaceColor', 'b', 'HandleVisibility','off');
else
    disp('Žiadne dáta na vykreslenie rýchlostí kolies pre Úlohu 2.');
end
title('Rýchlosti kolies');
xlabel('Čas (s)');
ylabel('Rýchlosť (m/s)');
grid on;
if ~isempty(t_input_task2) && Pocet_intervalov_task2 > 0
    xlim([t_input_task2(1) t_input_task2(end)]);
end
legend('show', 'Location', 'best');
hold off;

% Druhý podgraf: Rýchlosť ťažiska
subplot(2,1,2);
hold on;
if Pocet_intervalov_task2 > 0
    stairs([t_input_task2(1:Pocet_intervalov_task2), t_input_task2(end)], [v_T_hist_task2, v_T_hist_task2(end)], ...
           'g-.', 'LineWidth', 1.5, 'DisplayName', 'Rýchlosť ťažiska');
    plot(t_input_task2(1:Pocet_intervalov_task2) + diff(t_input_task2)/2, v_T_hist_task2, 'gs', 'MarkerSize', 3, 'MarkerFaceColor', 'g', 'HandleVisibility','off');
else
    disp('Žiadne dáta na vykreslenie rýchlosti ťažiska pre Úlohu 2.');
end
title('Rýchlosť ťažiska');
xlabel('Čas (s)');
ylabel('Rýchlosť (m/s)');
grid on;
if ~isempty(t_input_task2) && Pocet_intervalov_task2 > 0
    xlim([t_input_task2(1) t_input_task2(end)]);
end
legend('show', 'Location', 'best');
hold off;

sgtitle('Priebeh rýchlostí v čase (Štvorec - Úloha 2)');


% --- Vykreslenie trajektórií pre Úlohu 2 ---
figure('Name', 'Úloha 2: Trajektórie robota (Štvorec)');
if Pocet_v_Poli_task2 > 0
    plot(Hist_x_task2, Hist_y_task2, 'g-', 'LineWidth', 2, 'DisplayName', 'Ťažisko (P)');
    hold on;
    plot(x_L_hist_task2, y_L_hist_task2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ľavé koleso'); % Zmenené na 'r--' pre konzistenciu
    plot(x_R_hist_task2, y_R_hist_task2, 'b:', 'LineWidth', 1.5, 'DisplayName', 'Pravé koleso');
    plot(Hist_x_task2(1), Hist_y_task2(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'HandleVisibility','off');
    plot(x_L_hist_task2(1), y_L_hist_task2(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6, 'HandleVisibility','off');
    plot(x_R_hist_task2(1), y_R_hist_task2(1), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6, 'HandleVisibility','off');
    hold off;
else
    disp('Žiadne dáta na vykreslenie trajektórií pre Úlohu 2.');
end
title('Trajektórie ťažiska a kolies (Štvorec)');
xlabel('X (m)');
ylabel('Y (m)');
legend('show', 'Location', 'bestoutside');
axis equal;
grid on;

disp('Úloha 2 dokončená.');
