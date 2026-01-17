clear; clc; close all;
% Zadanie č.3 z predmetu Robotika - Úloha 1

% --- Parametre diferenciálneho podvozku ---
L = 0.2; % Rozchod kolies [m] (200 mm)
% r = 0.05; % Polomer kolesa [m] (50 mm) - nie je priamo potrebný pre túto úlohu,
            % ak sú rýchlosti kolies dané ako tangenciálne

% --- Vstupné vektory (príklad zo zadania) ---
t_input = [0 5 10 15 20]; % Časový vektor [s]
RLKI = [2 0 1 2 1]; % Rýchlosť ľavého kolesa [m/s]
RPKI = [2 1 1 -2 1]; % Rýchlosť pravého kolesa [m/s]

% --- Inicializácia ---
Pocet_v_Poli = length(t_input);
Pocet_intervalov = Pocet_v_Poli - 1;

% História stavu robota (ťažisko)
Hist_x = zeros(1, Pocet_v_Poli);
Hist_y = zeros(1, Pocet_v_Poli);
Hist_fi = zeros(1, Pocet_v_Poli); % Orientácia robota [rad]

% História polohy kolies
x_L_hist = zeros(1, Pocet_v_Poli);
y_L_hist = zeros(1, Pocet_v_Poli);
x_R_hist = zeros(1, Pocet_v_Poli);
y_R_hist = zeros(1, Pocet_v_Poli);

% História rýchlosti ťažiska (pre každý interval)
v_T_hist = zeros(1, Pocet_intervalov);

% Počiatočné podmienky (v čase t_input(1))
Hist_x(1) = 0;
Hist_y(1) = 0;
Hist_fi(1) = 0;

x_L_hist(1) = Hist_x(1) - (L/2) * sin(Hist_fi(1));
y_L_hist(1) = Hist_y(1) + (L/2) * cos(Hist_fi(1));
x_R_hist(1) = Hist_x(1) + (L/2) * sin(Hist_fi(1));
y_R_hist(1) = Hist_y(1) - (L/2) * cos(Hist_fi(1));

% --- Simulácia pohybu ---
for k = 1:Pocet_intervalov
    dt = t_input(k+1) - t_input(k);
    v_L = RLKI(k);
    v_R = RPKI(k);

    v_T = (v_R + v_L) / 2;
    omega = (v_R - v_L) / L;
    v_T_hist(k) = v_T;

    x_prev = Hist_x(k);
    y_prev = Hist_y(k);
    phi_prev = Hist_fi(k);

    Hist_x(k+1) = x_prev + v_T * cos(phi_prev) * dt;
    Hist_y(k+1) = y_prev + v_T * sin(phi_prev) * dt;
    Hist_fi(k+1) = phi_prev + omega * dt;
    Hist_fi(k+1) = mod(Hist_fi(k+1) + pi, 2*pi) - pi;

    phi_current = Hist_fi(k+1);
    x_T_current = Hist_x(k+1);
    y_T_current = Hist_y(k+1);

    x_L_hist(k+1) = x_T_current - (L/2) * sin(phi_current);
    y_L_hist(k+1) = y_T_current + (L/2) * cos(phi_current);
    x_R_hist(k+1) = x_T_current + (L/2) * sin(phi_current);
    y_R_hist(k+1) = y_T_current - (L/2) * cos(phi_current);
end

% --- Vykreslenie priebehov rýchlostí (VŠETKY NA JEDNOM GRAFE) ---
figure('Name', 'Priebehy rýchlostí v čase');
hold on;

% Rýchlosť ľavého kolesa
stairs(t_input, [RLKI(1:end-1), RLKI(end-1)],'r-', 'LineWidth', 1.5, 'DisplayName', 'Rýchlosť ľavého kolesa');
% Zmena tu: kreslíme značky len po predposledný bod
plot(t_input(1:end-1), RLKI(1:end-1), 'ro', 'MarkerFaceColor', 'r', 'HandleVisibility','off');

% Rýchlosť pravého kolesa
stairs(t_input, [RPKI(1:end-1), RPKI(end-1)],'b--', 'LineWidth', 1.5, 'DisplayName', 'Rýchlosť pravého kolesa');
% Zmena tu: kreslíme značky len po predposledný bod
plot(t_input(1:end-1), RPKI(1:end-1), 'bo', 'MarkerFaceColor', 'b', 'HandleVisibility','off');

% Rýchlosť ťažiska
stairs([t_input(1:end-1), t_input(end)], [v_T_hist, v_T_hist(end)], ...
       'g-.', 'LineWidth', 1.5, 'DisplayName', 'Rýchlosť ťažiska');
% Doplnené značky pre rýchlosť ťažiska (v strede intervalov)
plot(t_input(1:end-1) + diff(t_input)/2, v_T_hist, 'gs', 'MarkerFaceColor', 'g', 'HandleVisibility','off');

title('Priebeh rýchlostí v čase');
xlabel('Čas (s)');
ylabel('Rýchlosť (m/s)');
grid on;
xlim([t_input(1) t_input(end)]);
legend('show', 'Location', 'best');
hold off;

% --- Vykreslenie trajektórií ---
figure('Name', 'Trajektórie robota');
plot(Hist_x, Hist_y, 'g-', 'LineWidth', 2, 'DisplayName', 'Ťažisko (P)'); % Trajektória ťažiska je zelená
hold on;
plot(x_L_hist, y_L_hist, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ľavé koleso');
plot(x_R_hist, y_R_hist, 'b:', 'LineWidth', 1.5, 'DisplayName', 'Pravé koleso'); % Trajektória pravého kolesa je modrá

% Zvýraznenie štartovacích bodov
% Pre konzistenciu farieb:
plot(Hist_x(1), Hist_y(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'HandleVisibility','off'); % Zelený bod pre zelenú trajektóriu
plot(x_L_hist(1), y_L_hist(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6, 'HandleVisibility','off');
plot(x_R_hist(1), y_R_hist(1), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6, 'HandleVisibility','off'); % Modrý bod pre modrú trajektóriu

title('Trajektórie ťažiska a kolies');
xlabel('X (m)');
ylabel('Y (m)');
legend('show', 'Location', 'best');
axis equal;
grid on;
hold off;

disp('Simulácia dokončená a grafy vykreslené.');
