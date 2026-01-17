clear; clc; close all;

% =========================================================================
% ÚLOHA 3: Vykreslenie trajektórie krivka podľa Obr. 2
% =========================================================================
disp('--- Úloha 3: Krivka R1-L1-R2 ---');

% --- Parametre robota ---
L = 0.2; % Rozchod kolies [m]

% --- Používateľský vstup ---
R1_vstup = input('Zadajte polomer prvého oblúka R1 [m]: ');
if isempty(R1_vstup) || ~isnumeric(R1_vstup) || R1_vstup <= 0
    R1_vstup = 1.0;
    disp(['Neplatný vstup pre R1, použije sa predvolená hodnota: ', num2str(R1_vstup), ' m']);
end

L1_vstup = input('Zadajte dĺžku rovného úseku L1 [m]: ');
if isempty(L1_vstup) || ~isnumeric(L1_vstup) || L1_vstup < 0 % L1 môže byť 0
    L1_vstup = 1.0;
    disp(['Neplatný vstup pre L1, použije sa predvolená hodnota: ', num2str(L1_vstup), ' m']);
end

R2_vstup = input('Zadajte polomer druhého oblúka R2 [m]: ');
if isempty(R2_vstup) || ~isnumeric(R2_vstup) || R2_vstup <= 0
    R2_vstup = 1.0;
    disp(['Neplatný vstup pre R2, použije sa predvolená hodnota: ', num2str(R2_vstup), ' m']);
end

% --- Konštanty pre pohyb v Úlohe 3 ---
v_tangencial = 0.2;         % Tangenciálna rýchlosť ťažiska [m/s]
uhol_obluka = pi/2;         % Uhol každého oblúka (90 stupňov)
num_segments_per_arc = 30;  % Počet segmentov pre každý oblúk
num_segments_straight = 10; % Počet segmentov pre rovný úsek (ak L1 > 0)

% --- Generovanie časov a rýchlostí ---
t_current_task3 = 0;
t_input_task3 = [0]; % Začíname v čase 0
RLKI_task3 = [];
RPKI_task3 = [];

% 1. Oblúk 1 (polomer R1, doľava)
if R1_vstup > 0
    omega_T_arc1 = v_tangencial / R1_vstup; % Kladná pre otočenie doľava
    vR_arc1 = v_tangencial + (omega_T_arc1 * L / 2);
    vL_arc1 = v_tangencial - (omega_T_arc1 * L / 2);
    duration_arc1_total = (R1_vstup * uhol_obluka) / v_tangencial;
    dt_arc1_segment = duration_arc1_total / num_segments_per_arc;

    for seg = 1:num_segments_per_arc
        RLKI_task3 = [RLKI_task3, vL_arc1];
        RPKI_task3 = [RPKI_task3, vR_arc1];
        t_current_task3 = t_current_task3 + dt_arc1_segment;
        t_input_task3 = [t_input_task3, t_current_task3];
    end
end

% 2. Rovný úsek (dĺžka L1)
if L1_vstup > 0
    vL_straight = v_tangencial;
    vR_straight = v_tangencial;
    duration_straight_total = L1_vstup / v_tangencial;
    dt_straight_segment = duration_straight_total / num_segments_straight;

    for seg = 1:num_segments_straight
        RLKI_task3 = [RLKI_task3, vL_straight];
        RPKI_task3 = [RPKI_task3, vR_straight];
        t_current_task3 = t_current_task3 + dt_straight_segment;
        t_input_task3 = [t_input_task3, t_current_task3];
    end
end

% 3. Oblúk 2 (polomer R2, doprava)
if R2_vstup > 0
    omega_T_arc2 = -v_tangencial / R2_vstup; % Záporná pre otočenie doprava
    vR_arc2 = v_tangencial + (omega_T_arc2 * L / 2);
    vL_arc2 = v_tangencial - (omega_T_arc2 * L / 2);
    duration_arc2_total = (R2_vstup * uhol_obluka) / v_tangencial; % Dĺžka oblúka je kladná
    dt_arc2_segment = duration_arc2_total / num_segments_per_arc;

    for seg = 1:num_segments_per_arc
        RLKI_task3 = [RLKI_task3, vL_arc2];
        RPKI_task3 = [RPKI_task3, vR_arc2];
        t_current_task3 = t_current_task3 + dt_arc2_segment;
        t_input_task3 = [t_input_task3, t_current_task3];
    end
end

% --- Inicializácia pre simuláciu Úlohy 3 ---
Pocet_intervalov_task3 = length(RLKI_task3);
Pocet_v_Poli_task3 = length(t_input_task3);

Hist_x_task3 = zeros(1, Pocet_v_Poli_task3);
Hist_y_task3 = zeros(1, Pocet_v_Poli_task3);
Hist_fi_task3 = zeros(1, Pocet_v_Poli_task3);

x_L_hist_task3 = zeros(1, Pocet_v_Poli_task3);
y_L_hist_task3 = zeros(1, Pocet_v_Poli_task3);
x_R_hist_task3 = zeros(1, Pocet_v_Poli_task3);
y_R_hist_task3 = zeros(1, Pocet_v_Poli_task3);

v_T_hist_task3 = zeros(1, Pocet_intervalov_task3);

Hist_x_task3(1) = 0;
Hist_y_task3(1) = 0;
Hist_fi_task3(1) = 0; % Začíname orientovaní pozdĺž osi X

x_L_hist_task3(1) = Hist_x_task3(1) - (L/2) * sin(Hist_fi_task3(1));
y_L_hist_task3(1) = Hist_y_task3(1) + (L/2) * cos(Hist_fi_task3(1));
x_R_hist_task3(1) = Hist_x_task3(1) + (L/2) * sin(Hist_fi_task3(1));
y_R_hist_task3(1) = Hist_y_task3(1) - (L/2) * cos(Hist_fi_task3(1));

% --- Simulácia pohybu pre Úlohu 3 ---
if Pocet_intervalov_task3 > 0
    for k = 1:Pocet_intervalov_task3
        dt_task3 = t_input_task3(k+1) - t_input_task3(k);
        v_L_task3 = RLKI_task3(k);
        v_R_task3 = RPKI_task3(k);

        v_T_val_task3 = (v_R_task3 + v_L_task3) / 2;
        omega_val_task3 = (v_R_task3 - v_L_task3) / L;
        v_T_hist_task3(k) = v_T_val_task3;

        x_prev_task3 = Hist_x_task3(k);
        y_prev_task3 = Hist_y_task3(k);
        phi_prev_task3 = Hist_fi_task3(k);

        Hist_x_task3(k+1) = x_prev_task3 + v_T_val_task3 * cos(phi_prev_task3) * dt_task3;
        Hist_y_task3(k+1) = y_prev_task3 + v_T_val_task3 * sin(phi_prev_task3) * dt_task3;
        Hist_fi_task3(k+1) = phi_prev_task3 + omega_val_task3 * dt_task3;
        Hist_fi_task3(k+1) = mod(Hist_fi_task3(k+1) + pi, 2*pi) - pi;

        phi_current_task3 = Hist_fi_task3(k+1);
        x_T_current_task3 = Hist_x_task3(k+1);
        y_T_current_task3 = Hist_y_task3(k+1);

        x_L_hist_task3(k+1) = x_T_current_task3 - (L/2) * sin(phi_current_task3);
        y_L_hist_task3(k+1) = y_T_current_task3 + (L/2) * cos(phi_current_task3);
        x_R_hist_task3(k+1) = x_T_current_task3 + (L/2) * sin(phi_current_task3);
        y_R_hist_task3(k+1) = y_T_current_task3 - (L/2) * cos(phi_current_task3);
    end
end

% --- Vykreslenie všetkých grafov pre Úlohu 3 v jednom okne ---
figure('Name', 'Úloha 3: Kompletný prehľad (Krivka R1-L1-R2)');

% Prvý podgraf (ľavý, väčší): Trajektórie
subplot(2,3,[1 2 4 5]);
if Pocet_v_Poli_task3 > 0
    plot(Hist_x_task3, Hist_y_task3, 'g-', 'LineWidth', 2, 'DisplayName', 'Ťažisko (P)');
    hold on;
    plot(x_L_hist_task3, y_L_hist_task3, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ľavé koleso');
    plot(x_R_hist_task3, y_R_hist_task3, 'b:', 'LineWidth', 1.5, 'DisplayName', 'Pravé koleso');
    plot(Hist_x_task3(1), Hist_y_task3(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'HandleVisibility','off');
    plot(x_L_hist_task3(1), y_L_hist_task3(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6, 'HandleVisibility','off');
    plot(x_R_hist_task3(1), y_R_hist_task3(1), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6, 'HandleVisibility','off');
    hold off;
else
    disp('Žiadne dáta na vykreslenie trajektórií pre Úlohu 3.');
end
title('Trajektórie ťažiska a kolies (Krivka R1-L1-R2)');
xlabel('X (m)');
ylabel('Y (m)');
legend('show', 'Location', 'bestoutside');
axis equal; % Dôležité pre správne zobrazenie kruhových oblúkov
grid on;
% Automatické nastavenie limitov pre trajektóriu môže byť lepšie,
% alebo môžete pridať padding:
if Pocet_v_Poli_task3 > 0
    padding = max(R1_vstup, R2_vstup) * 0.2 + L1_vstup * 0.1 + 0.5; % Príklad paddingu
    xlim([min(Hist_x_task3)-padding, max(Hist_x_task3)+padding]);
    ylim([min(Hist_y_task3)-padding, max(Hist_y_task3)+padding]);
end


% Druhý podgraf (horný pravý, menší): Rýchlosti kolies
subplot(2,3,3);
hold on;
if Pocet_intervalov_task3 > 0
    stairs(t_input_task3, [RLKI_task3, RLKI_task3(end)],'r-', 'LineWidth', 1.5, 'DisplayName', 'Rýchlosť ľavého kolesa');
    plot(t_input_task3(1:Pocet_intervalov_task3), RLKI_task3, 'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r', 'HandleVisibility','off');

    stairs(t_input_task3, [RPKI_task3, RPKI_task3(end)],'b--', 'LineWidth', 1.5, 'DisplayName', 'Rýchlosť pravého kolesa');
    plot(t_input_task3(1:Pocet_intervalov_task3), RPKI_task3, 'bo', 'MarkerSize', 3, 'MarkerFaceColor', 'b', 'HandleVisibility','off');
else
    disp('Žiadne dáta na vykreslenie rýchlostí kolies pre Úlohu 3.');
end
title('Rýchlosti kolies');
xlabel('Čas (s)');
ylabel('Rýchlosť (m/s)');
grid on;
if ~isempty(t_input_task3) && Pocet_intervalov_task3 > 0
    xlim([t_input_task3(1) t_input_task3(end)]);
end
legend('show', 'Location', 'best');
hold off;

% Tretí podgraf (dolný pravý, menší): Rýchlosť ťažiska
subplot(2,3,6);
hold on;
if Pocet_intervalov_task3 > 0
    stairs([t_input_task3(1:Pocet_intervalov_task3), t_input_task3(end)], [v_T_hist_task3, v_T_hist_task3(end)], ...
           'g-.', 'LineWidth', 1.5, 'DisplayName', 'Rýchlosť ťažiska');
    plot(t_input_task3(1:Pocet_intervalov_task3) + diff(t_input_task3)/2, v_T_hist_task3, 'gs', 'MarkerSize', 3, 'MarkerFaceColor', 'g', 'HandleVisibility','off');
else
    disp('Žiadne dáta na vykreslenie rýchlosti ťažiska pre Úlohu 3.');
end
title('Rýchlosť ťažiska');
xlabel('Čas (s)');
ylabel('Rýchlosť (m/s)');
grid on;
if ~isempty(t_input_task3) && Pocet_intervalov_task3 > 0
    xlim([t_input_task3(1) t_input_task3(end)]);
end
legend('show', 'Location', 'best');
hold off;

sgtitle('Kompletný prehľad pre Úlohu 3: Krivka R1-L1-R2');

disp('Úloha 3 dokončená.');
