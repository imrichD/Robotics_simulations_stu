clear; clc; close all;

% =========================================================================
% ÚLOHA 4: Interaktívne ovládanie robota
% =========================================================================
disp('--- Úloha 4: Interaktívna hra ---');
disp('Ovládanie: W/S/A/D alebo Šípky. ESC na ukončenie.');

% --- Parametre robota a simulácie ---
L = 0.2;                % Rozchod kolies [m]
max_v_lin = 0.5;        % Max lineárna rýchlosť [m/s]
max_omega_rot = pi;     % Max uhlová rýchlosť [rad/s] (180 deg/s)
dt_sim = 0.05;          % Časový krok simulácie [s]

% --- Globálne premenné pre stav kláves ---
global keyStates;
keyStates.forward = 0;
keyStates.backward = 0;
keyStates.left = 0;
keyStates.right = 0;
keyStates.quit = 0;

% --- Inicializácia pre ukladanie histórie ---
t_hist_task4 = [0];
RLKI_hist_task4 = []; 
RPKI_hist_task4 = []; 
v_T_val_hist_task4 = [];

% Počiatočná pozícia a orientácia
phi_initial = 0;
x_initial = 0;
y_initial = 0;

Hist_x_task4 = [x_initial]; 
Hist_y_task4 = [y_initial];
Hist_fi_task4 = [phi_initial];
x_L_hist_task4 = [x_initial - (L/2)*sin(phi_initial)];
y_L_hist_task4 = [y_initial + (L/2)*cos(phi_initial)];
x_R_hist_task4 = [x_initial + (L/2)*sin(phi_initial)];
y_R_hist_task4 = [y_initial - (L/2)*cos(phi_initial)];

% --- Nastavenie Figure a Callback funkcií ---
fig_handle = figure('Name', 'Úloha 4: Ovládanie Robota', ...
                    'KeyPressFcn', @keyDownListener, ...
                    'KeyReleaseFcn', @keyUpListener, ...
                    'CloseRequestFcn', @closeGameListener);

ax_robot = axes('Parent', fig_handle, 'NextPlot', 'replacechildren');
axis(ax_robot, 'equal');
grid(ax_robot, 'on');
xlabel(ax_robot, 'X (m)');
ylabel(ax_robot, 'Y (m)');
title(ax_robot, 'Dynamická trajektória - Ovládajte robota');
initial_axis_limit = 1.0; % Počiatočný rozsah osí
xlim(ax_robot, [-initial_axis_limit initial_axis_limit]);
ylim(ax_robot, [-initial_axis_limit initial_axis_limit]);

% --- Hlavná slučka hry ---
currentTime = 0;

disp('Hra spustená. Ovládajte robota a stlačte ESC alebo zatvorte okno pre ukončenie.');

while ~keyStates.quit
    % 1. Určenie cieľových rýchlostí na základe stlačených kláves
    target_v_lin = 0;
    target_omega_input = 0; % Uhlová rýchlosť zadaná používateľom

    if keyStates.forward
        target_v_lin = target_v_lin + max_v_lin;
    end
    if keyStates.backward
        target_v_lin = target_v_lin - max_v_lin;
    end
    if keyStates.left % Otáčanie doľava -> kladná omega
        target_omega_input = target_omega_input + max_omega_rot;
    end
    if keyStates.right % Otáčanie doprava -> záporná omega
        target_omega_input = target_omega_input - max_omega_rot;
    end

    % Výpočet rýchlostí kolies z target_v_lin a target_omega_input
    % target_omega_input je priamo uhlová rýchlosť robota, nie kolesa
    v_R_current = target_v_lin + (target_omega_input * L / 2);
    v_L_current = target_v_lin - (target_omega_input * L / 2);

    % 2. Simulácia kroku
    x_prev = Hist_x_task4(end);
    y_prev = Hist_y_task4(end);
    phi_prev = Hist_fi_task4(end);

    % Skutočné rýchlosti ťažiska a otáčania na základe rýchlostí kolies
    v_T_actual = (v_R_current + v_L_current) / 2;
    omega_actual = (v_R_current - v_L_current) / L;

    x_new = x_prev + v_T_actual * cos(phi_prev) * dt_sim;
    y_new = y_prev + v_T_actual * sin(phi_prev) * dt_sim;
    phi_new = phi_prev + omega_actual * dt_sim;
    phi_new = mod(phi_new + pi, 2*pi) - pi;

    % Výpočet pozícií kolies
    x_L_new = x_new - (L/2) * sin(phi_new);
    y_L_new = y_new + (L/2) * cos(phi_new);
    x_R_new = x_new + (L/2) * sin(phi_new);
    y_R_new = y_new - (L/2) * cos(phi_new);

    % 3. Ukladanie histórie
    currentTime = currentTime + dt_sim;
    t_hist_task4 = [t_hist_task4, currentTime];
    
    RLKI_hist_task4 = [RLKI_hist_task4, v_L_current];
    RPKI_hist_task4 = [RPKI_hist_task4, v_R_current];
    v_T_val_hist_task4 = [v_T_val_hist_task4, v_T_actual];

    Hist_x_task4 = [Hist_x_task4, x_new];
    Hist_y_task4 = [Hist_y_task4, y_new];
    Hist_fi_task4 = [Hist_fi_task4, phi_new];
    x_L_hist_task4 = [x_L_hist_task4, x_L_new];
    y_L_hist_task4 = [y_L_hist_task4, y_L_new];
    x_R_hist_task4 = [x_R_hist_task4, x_R_new];
    y_R_hist_task4 = [y_R_hist_task4, y_R_new];

    % 4. Dynamické vykresľovanie
    cla(ax_robot); 
    hold(ax_robot, 'on');
    plot(ax_robot, Hist_x_task4, Hist_y_task4, 'g-'); % Trajektória ťažiska
    
    % Vykreslenie aktuálnych pozícií ako bodov pre debugovanie geometrie
    plot(ax_robot, x_new, y_new, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Ťažisko
    plot(ax_robot, x_L_new, y_L_new, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r'); % Ľavé koleso
    plot(ax_robot, x_R_new, y_R_new, 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b'); % Pravé koleso

    % Spojnica medzi kolesami (oska)
    plot(ax_robot, [x_L_new, x_R_new], [y_L_new, y_R_new], 'k-', 'LineWidth',1); 
    % Čiara od ťažiska k stredu osky (mala by byť kolmá na osku a ukazovať dopredu)
    plot(ax_robot, [x_new, x_new + L/1.5 * cos(phi_new)], ...
                   [y_new, y_new + L/1.5 * sin(phi_new)], 'm-', 'LineWidth', 2);

    hold(ax_robot, 'off');
    
    % Prispôsobenie limitov osí
    padding_dynamic = L*3; % Väčší padding pre lepšiu viditeľnosť
    min_x_data = min(Hist_x_task4); max_x_data = max(Hist_x_task4);
    min_y_data = min(Hist_y_task4); max_y_data = max(Hist_y_task4);
    
    current_xlim_val = [min_x_data - padding_dynamic, max_x_data + padding_dynamic];
    current_ylim_val = [min_y_data - padding_dynamic, max_y_data + padding_dynamic];
    
    % Zabezpečenie minimálnej veľkosti okna
    if diff(current_xlim_val) < 2*initial_axis_limit
        center_x = (current_xlim_val(1)+current_xlim_val(2))/2;
        current_xlim_val = [center_x - initial_axis_limit, center_x + initial_axis_limit];
    end
    if diff(current_ylim_val) < 2*initial_axis_limit
        center_y = (current_ylim_val(1)+current_ylim_val(2))/2;
        current_ylim_val = [center_y - initial_axis_limit, center_y + initial_axis_limit];
    end
    
    xlim(ax_robot, current_xlim_val);
    ylim(ax_robot, current_ylim_val);
    axis(ax_robot, 'equal');
    
    drawnow;

    % 5. Pauza
    pause(dt_sim);
end

disp('Hra ukončená. Vykresľujem finálne grafy...');

% --- Vykreslenie finálnych grafov po ukončení hry ---
if length(t_hist_task4) > 1 
    Pocet_intervalov_task4 = length(RLKI_hist_task4); % Malo by byť length(t_hist_task4) - 1

    figure('Name', 'Úloha 4: Finálne Grafy');
    % Prvý podgraf (ľavý, väčší): Trajektórie
    subplot(2,3,[1 2 4 5]);
    plot(Hist_x_task4, Hist_y_task4, 'g-', 'LineWidth', 2, 'DisplayName', 'Ťažisko (P)');
    hold on;
    plot(x_L_hist_task4, y_L_hist_task4, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ľavé koleso');
    plot(x_R_hist_task4, y_R_hist_task4, 'b:', 'LineWidth', 1.5, 'DisplayName', 'Pravé koleso');
    plot(Hist_x_task4(1), Hist_y_task4(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'HandleVisibility','off');
    plot(x_L_hist_task4(1), y_L_hist_task4(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6, 'HandleVisibility','off');
    plot(x_R_hist_task4(1), y_R_hist_task4(1), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6, 'HandleVisibility','off');
    hold off;
    title('Finálna Trajektória ťažiska a kolies');
    xlabel('X (m)'); ylabel('Y (m)');
    legend('show', 'Location', 'bestoutside'); axis equal; grid on;
    
    % Nastavenie limitov pre finálny graf trajektórií
    padding_final = L + 0.5; % Menší padding pre finálny graf
    final_xlim = [min(Hist_x_task4)-padding_final, max(Hist_x_task4)+padding_final];
    final_ylim = [min(Hist_y_task4)-padding_final, max(Hist_y_task4)+padding_final];
    if diff(final_xlim) < 2*initial_axis_limit; center_x = mean(final_xlim); final_xlim = [center_x - initial_axis_limit, center_x + initial_axis_limit]; end
    if diff(final_ylim) < 2*initial_axis_limit; center_y = mean(final_ylim); final_ylim = [center_y - initial_axis_limit, center_y + initial_axis_limit]; end
    xlim(final_xlim); ylim(final_ylim);


    % Druhý podgraf (horný pravý, menší): Rýchlosti kolies
    subplot(2,3,3);
    hold on;
    if Pocet_intervalov_task4 > 0
        stairs(t_hist_task4(1:Pocet_intervalov_task4), RLKI_hist_task4,'r-', 'LineWidth', 1.5, 'DisplayName', 'Rýchlosť ľavého kolesa');
        stairs(t_hist_task4(1:Pocet_intervalov_task4), RPKI_hist_task4,'b--', 'LineWidth', 1.5, 'DisplayName', 'Rýchlosť pravého kolesa');
    end
    title('Rýchlosti kolies v čase');
    xlabel('Čas (s)'); ylabel('Rýchlosť (m/s)'); grid on;
    if Pocet_intervalov_task4 > 0; xlim([t_hist_task4(1) t_hist_task4(Pocet_intervalov_task4)]); end
    legend('show', 'Location', 'best');
    hold off;

    % Tretí podgraf (dolný pravý, menší): Rýchlosť ťažiska
    subplot(2,3,6);
    hold on;
    if Pocet_intervalov_task4 > 0
        stairs(t_hist_task4(1:Pocet_intervalov_task4), v_T_val_hist_task4, 'g-.', 'LineWidth', 1.5, 'DisplayName', 'Rýchlosť ťažiska');
    end
    title('Rýchlosť ťažiska v čase');
    xlabel('Čas (s)'); ylabel('Rýchlosť (m/s)'); grid on;
    if Pocet_intervalov_task4 > 0; xlim([t_hist_task4(1) t_hist_task4(Pocet_intervalov_task4)]); end
    legend('show', 'Location', 'best');
    hold off;
    
    sgtitle('Finálny prehľad pre Úlohu 4: Interaktívna Hra');
else
    disp('Žiadny pohyb nebol zaznamenaný na vykreslenie finálnych grafov.');
    if ishandle(fig_handle) % Ak okno hry ešte existuje, zatvor ho
        close(fig_handle);
    end
end

disp('Úloha 4 dokončená.');

% --- Callback funkcie pre klávesy ---
function keyDownListener(~, event)
    global keyStates;
    switch event.Key
        case {'w', 'uparrow'}
            keyStates.forward = 1;
        case {'s', 'downarrow'}
            keyStates.backward = 1;
        case {'a', 'leftarrow'}
            keyStates.left = 1;
        case {'d', 'rightarrow'}
            keyStates.right = 1;
        case 'escape'
            keyStates.quit = 1;
    end
end

function keyUpListener(~, event)
    global keyStates;
    switch event.Key
        case {'w', 'uparrow'}
            keyStates.forward = 0;
        case {'s', 'downarrow'}
            keyStates.backward = 0;
        case {'a', 'leftarrow'}
            keyStates.left = 0;
        case {'d', 'rightarrow'}
            keyStates.right = 0;
    end
end

function closeGameListener(src, ~)
    global keyStates;
    keyStates.quit = 1;
    if ishandle(src) % Skontroluj, či handle stále existuje
        delete(src); 
    end
    disp('Okno hry zatvorené používateľom.');
end
