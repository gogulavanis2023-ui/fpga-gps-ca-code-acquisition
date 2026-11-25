clear; clc; close all;
%% ---- CONSTANTS ----
c        = 299792458;       % Speed of light (m/s)
f_L1     = 1575.42e6;       % GPS L1 frequency (Hz)
f_L2     = 1227.60e6;       % GPS L2 frequency (Hz)
lambda_L1 = c / f_L1;       % L1 wavelength (m)
lambda_L2 = c / f_L2;       % L2 wavelength (m)
R_earth  = 6371000;         % Earth radius (m)
a_WGS84  = 6378137.0;       % WGS84 semi-major axis (m)
e2_WGS84 = 0.00669437999014;% WGS84 first eccentricity squared
wR  = 0.98;         % Radial weight
wAC = 1/sqrt(49);   % Along-track / Cross-track weight
fprintf('========================================\n');
fprintf('  GPS PSEUDORANGE AND SISRE CALCULATOR  \n');
fprintf('========================================\n\n');
%% ---- STEP 1: DEFINE RECEIVER POSITION ----
% Chennai, India: Lat=13.08N, Lon=80.27E, Alt=10m
lat_r = 13.08 * pi/180;
lon_r = 80.27 * pi/180;
alt_r = 10.0;
N_r   = a_WGS84 / sqrt(1 - e2_WGS84 * sin(lat_r)^2);
x_r   = (N_r + alt_r) * cos(lat_r) * cos(lon_r);
y_r   = (N_r + alt_r) * cos(lat_r) * sin(lon_r);
z_r   = (N_r*(1 - e2_WGS84) + alt_r) * sin(lat_r);
r_rx  = [x_r; y_r; z_r];
fprintf('RECEIVER POSITION\n');
fprintf('  Location:  Chennai, India (%.2fN, %.2fE)\n', ...
        lat_r*180/pi, lon_r*180/pi);
fprintf('  ECEF:      X=%.0f m, Y=%.0f m, Z=%.0f m\n\n', x_r, y_r, z_r);
%% ---- STEP 2: DEFINE GPS SATELLITE POSITIONS ----
n_sat = 6;
el_deg = [65,  42,  78,  25,  55,  38];   % degrees
az_deg = [45, 135, 230, 310,  90, 200];   % degrees
el     = el_deg * pi/180;
az     = az_deg * pi/180;
R_orbit = R_earth + 20200e3;
R_ENU2ECEF = [-sin(lon_r),           cos(lon_r),          0; ...
              -sin(lat_r)*cos(lon_r), -sin(lat_r)*sin(lon_r), cos(lat_r); ...
               cos(lat_r)*cos(lon_r),  cos(lat_r)*sin(lon_r), sin(lat_r)];
sat_pos_true = zeros(3, n_sat);  % "true" positions (from precise ephemeris)
sat_pos_bcast = zeros(3, n_sat); % broadcast positions (with orbit errors)
mu_earth    = 3.986005e14;
v_orbital   = sqrt(mu_earth / R_orbit);
sigma_R = 0.18;   % Radial orbit error (m)
sigma_A = 1.05;   % Along-track orbit error (m)
sigma_C = 0.44;   % Cross-track orbit error (m)
sigma_clk_sat = 0.69 / c;  % Convert m to seconds
sat_vel     = zeros(3, n_sat);
dR          = zeros(n_sat, 1);  % Radial orbit errors
dA          = zeros(n_sat, 1);  % Along-track orbit errors
dC          = zeros(n_sat, 1);  % Cross-track orbit errors
dt_s_true   = zeros(n_sat, 1);  % True satellite clock offset
dt_s_bcast  = zeros(n_sat, 1);  % Broadcast satellite clock (estimated)
rng(42); % Fixed seed for reproducibility
for i = 1:n_sat
    e_ENU = [cos(el(i))*sin(az(i)); ...
             cos(el(i))*cos(az(i)); ...
             sin(el(i))];
    e_ECEF = R_ENU2ECEF * e_ENU;
    range_to_sat = -R_earth*sin(el(i)) + ...
                    sqrt((R_earth*sin(el(i)))^2 + R_orbit^2 - R_earth^2);
    sat_pos_true(:,i) = r_rx + range_to_sat * e_ECEF;    
    pos_unit = sat_pos_true(:,i) / norm(sat_pos_true(:,i));
    v_dir    = cross([0;0;1], pos_unit);
    if norm(v_dir) > 0
        v_dir = v_dir / norm(v_dir);
    end
    sat_vel(:,i) = v_orbital * v_dir;
    dR(i) = sigma_R * randn();
    dA(i) = sigma_A * randn();
    dC(i) = sigma_C * randn();
    r_unit = pos_unit;                              % Radial
    a_unit = sat_vel(:,i)/norm(sat_vel(:,i));       % Along-track
    c_unit = cross(r_unit, a_unit);                 % Cross-track
    if norm(c_unit) > 0
        c_unit = c_unit / norm(c_unit);
    end
    orbit_error_ECEF = dR(i)*r_unit + dA(i)*a_unit + dC(i)*c_unit;
    sat_pos_bcast(:,i) = sat_pos_true(:,i) + orbit_error_ECEF;
    dt_s_true(i) = sigma_clk_sat * randn();
    sigma_clk_err = 0.5 / c;  % ~0.5 m clock estimation error
    dt_s_bcast(i) = dt_s_true(i) + sigma_clk_err * randn();
end
%% ---- STEP 3: COMPUTE TRUE GEOMETRIC RANGE ----
rho_true  = zeros(n_sat, 1);  % Range using precise ephemeris
rho_bcast = zeros(n_sat, 1);  % Range using broadcast ephemeris
for i = 1:n_sat
    rho_true(i)  = norm(sat_pos_true(:,i)  - r_rx);
    rho_bcast(i) = norm(sat_pos_bcast(:,i) - r_rx);
end
fprintf('GEOMETRIC RANGES\n');
fprintf('  %-6s %12s %12s %12s\n', 'Sat', 'True(km)', 'Bcast(km)', 'Diff(m)');
fprintf('  %s\n', repmat('-',1,46));
for i = 1:n_sat
    fprintf('  G%-5d %12.3f %12.3f %12.4f\n', i, ...
        rho_true(i)/1000, rho_bcast(i)/1000, ...
        rho_bcast(i)-rho_true(i));
end
fprintf('\n');
%% ---- STEP 4: MODEL ALL PSEUDORANGE ERROR SOURCES ----
dt_r = 0.8e-3;  % seconds
T_delay = zeros(n_sat, 1);
P0 = 1013.25;  % Surface pressure (hPa)
for i = 1:n_sat
    % Elevation mapping function: delays increase at low elevation
    T_zenith = 2.3 * exp(-0.116e-3 * alt_r);
    T_delay(i) = T_zenith / sin(el(i) + 0.00143/(el(i) + 0.0445));
end
TEC = 30;  % TECU (typical daytime, equatorial India)
I_L1 = zeros(n_sat, 1);
I_L2 = zeros(n_sat, 1);
for i = 1:n_sat
    I_vert_L1 = 40.3e16 * TEC / f_L1^2;
    I_vert_L2 = 40.3e16 * TEC / f_L2^2;
    F_iono = 1 / sqrt(1 - (R_earth*cos(el(i)) / (R_earth + 350e3))^2);
    I_L1(i) = I_vert_L1 * F_iono;
    I_L2(i) = I_vert_L2 * F_iono;
end
alpha_IF = f_L1^2 / (f_L1^2 - f_L2^2);
beta_IF  = f_L2^2 / (f_L1^2 - f_L2^2);
M = sigma_mp * randn(n_sat, 1);
sigma_noise_rx = 0.3;
epsilon = sigma_noise_rx * randn(n_sat, 1);
%% ---- STEP 5: COMPUTE PSEUDORANGES ----
PR_L1  = zeros(n_sat, 1);  % L1 pseudorange
PR_L2  = zeros(n_sat, 1);  % L2 pseudorange
PR_IF  = zeros(n_sat, 1);  % Ionosphere-free combination
for i = 1:n_sat
    PR_L1(i) = rho_bcast(i) ...          % geometric range (from broadcast sat pos)
             + c * dt_r ...               % receiver clock bias (+)
             - c * dt_s_bcast(i) ...      % satellite clock correction (-)
             + T_delay(i) ...             % troposphere (+)
             + I_L1(i) ...               % ionosphere (+) on L1
             + M(i) ...                  % multipath
             + epsilon(i);               % receiver noise
    PR_L2(i) = rho_bcast(i) ...
             + c * dt_r ...
             - c * dt_s_bcast(i) ...
             + T_delay(i) ...
             + I_L2(i) ...               % different ionosphere on L2
             + M(i) + sigma_noise_rx * randn();
    PR_IF(i) = alpha_IF * PR_L1(i) - beta_IF * PR_L2(i);
end
PR_true = zeros(n_sat, 1);
for i = 1:n_sat
    PR_true(i) = rho_true(i) + c*dt_r - c*dt_s_true(i) + T_delay(i) + I_L1(i);
end
PR_error = PR_L1 - PR_true;
fprintf('PSEUDORANGE MEASUREMENTS (L1)\n');
fprintf('  %-5s %6s %10s %10s %10s %10s %10s %10s\n', ...
        'Sat','El(°)','True(m)','Bcast(m)','Error(m)','T_tropo(m)','I_iono(m)','Multipath(m)');
fprintf('  %s\n', repmat('-',1,85));
for i = 1:n_sat
    fprintf('  G%-4d %6.1f %10.3f %10.3f %10.4f %10.3f %10.3f %10.4f\n', ...
        i, el_deg(i), PR_true(i), PR_L1(i), PR_error(i), ...
        T_delay(i), I_L1(i), M(i));
end
fprintf('\n');
%% ---- STEP 6: COMPUTE SISRE ----
SISRE_orb  = zeros(n_sat, 1);  % Orbit-only SISRE
SISRE_full = zeros(n_sat, 1);  % Full SISRE (orbit + clock)
SISRE_proj = zeros(n_sat, 1);  % SISRE projected onto actual LOS
dclk_m = c * (dt_s_true - dt_s_bcast);  % positive = broadcast clock runs fast
for i = 1:n_sat
    SISRE_orb(i) = sqrt(wR^2 * dR(i)^2 + wAC^2 * (dA(i)^2 + dC(i)^2));
    radial_clock_combo = wR * dR(i) - dclk_m(i);
    SISRE_full(i) = sqrt(radial_clock_combo^2 + wAC^2*(dA(i)^2 + dC(i)^2));
    diff     = sat_pos_true(:,i) - r_rx;
    e_LOS    = diff / norm(diff);   % unit LOS vector
    r_unit = sat_pos_true(:,i) / norm(sat_pos_true(:,i));
    a_unit = sat_vel(:,i) / norm(sat_vel(:,i));
    c_unit = cross(r_unit, a_unit);
    if norm(c_unit) > 0, c_unit = c_unit/norm(c_unit); end
    orbit_err_ECEF = dR(i)*r_unit + dA(i)*a_unit + dC(i)*c_unit;
    orbit_proj = dot(orbit_err_ECEF, e_LOS);
    SISRE_proj(i) = orbit_proj - dclk_m(i);
end
SISRE_orb_rms  = sqrt(mean(SISRE_orb.^2));
SISRE_full_rms = sqrt(mean(SISRE_full.^2));
SISRE_proj_rms = sqrt(mean(SISRE_proj.^2));
fprintf('SISRE ANALYSIS\n');
fprintf('  Weight factors: wR=%.2f, wAC=%.4f (1/sqrt(49))\n', wR, wAC);
fprintf('  Orbit errors (simulated from Montenbruck 2015 Table 3):\n');
fprintf('    sigma_R=%.2fm  sigma_A=%.2fm  sigma_C=%.2fm\n\n', ...
        sigma_R, sigma_A, sigma_C);
fprintf('  %-5s %6s %8s %8s %8s %10s %10s %10s\n', ...
        'Sat','El(°)','dR(m)','dA(m)','dC(m)', ...
        'dClk(m)','SISRE_orb','SISRE_full');
fprintf('  %s\n', repmat('-',1,75));
for i = 1:n_sat
    fprintf('  G%-4d %6.1f %8.4f %8.4f %8.4f %10.4f %10.4f %10.4f\n', ...
        i, el_deg(i), dR(i), dA(i), dC(i), ...
        dclk_m(i), SISRE_orb(i), SISRE_full(i));
end
fprintf('  %s\n', repmat('-',1,75));
fprintf('  %-5s %6s %8s %8s %8s %10s %10.4f %10.4f\n', ...
        'RMS','','','','','', SISRE_orb_rms, SISRE_full_rms);
fprintf('\n');
fprintf('CONSTELLATION SISRE SUMMARY\n');
fprintf('  SISRE (orbit only):        %.4f m\n', SISRE_orb_rms);
fprintf('  SISRE (orbit + clock):     %.4f m\n', SISRE_full_rms);
fprintf('  SISRE (projected on LOS):  %.4f m\n', SISRE_proj_rms);
fprintf('  Reference (Montenbruck 2015): 0.71 m (GPS, 2013 constellation)\n\n');

%% ---- STEP 7: PSEUDORANGE ERROR BUDGET SUMMARY ----
fprintf('ERROR BUDGET SUMMARY\n');
fprintf('  %-20s %8s %8s %8s\n', 'Error Source', 'Min(m)', 'Max(m)', 'RMS(m)');
orbit_LOS = zeros(n_sat,1);
for i = 1:n_sat
    r_unit = sat_pos_true(:,i) / norm(sat_pos_true(:,i));
    a_unit = sat_vel(:,i) / norm(sat_vel(:,i));
    c_u    = cross(r_unit, a_unit); if norm(c_u)>0, c_u=c_u/norm(c_u); end
    diff   = sat_pos_true(:,i) - r_rx;
    e_LOS  = diff/norm(diff);
    err_ECEF = dR(i)*r_unit + dA(i)*a_unit + dC(i)*c_u;
    orbit_LOS(i) = dot(err_ECEF, e_LOS);
end

errors = {orbit_LOS,   'Orbit (LOS proj)'; ...
          dclk_m,      'Sat clock error'; ...
          T_delay,     'Troposphere'; ...
          I_L1,        'Ionosphere (L1)'; ...
          M,           'Multipath'; ...
          epsilon,     'Receiver noise'};

for k = 1:size(errors,1)
    vals = errors{k,1};
    fprintf('  %-20s %8.3f %8.3f %8.3f\n', errors{k,2}, ...
        min(vals), max(vals), sqrt(mean(vals.^2)));
end
fprintf('  %s\n', repmat('-',1,48));
fprintf('  %-20s %8s %8s %8.3f\n', 'TOTAL (PR error)', '', '', ...
        sqrt(mean(PR_error.^2)));
fprintf('\n');

%% ---- STEP 8: WORST USER LOCATION SISRE ----
fprintf('WORST USER LOCATION (WUL) SISRE\n');

SISRE_wul = zeros(n_sat, 1);
for i = 1:n_sat
    % Orbit error vector in ECEF
    r_unit = sat_pos_true(:,i) / norm(sat_pos_true(:,i));
    a_unit = sat_vel(:,i) / norm(sat_vel(:,i));
    c_u    = cross(r_unit, a_unit); if norm(c_u)>0, c_u=c_u/norm(c_u); end
    Dr_ECEF = dR(i)*r_unit + dA(i)*a_unit + dC(i)*c_u;
    
    % Magnitude of orbit error
    Dr_mag = norm(Dr_ECEF);
    
    % Direction of orbit error relative to nadir
    % beta = arcsin(R_earth / R_orbit) = apparent semi-diameter of Earth
    beta = asin(R_earth / norm(sat_pos_true(:,i)));
    
    % Angle between orbit error and nadir direction
    sat_nadir = -sat_pos_true(:,i) / norm(sat_pos_true(:,i)); % toward Earth center
    cos_angle = dot(Dr_ECEF, sat_nadir) / (Dr_mag + eps);
    angle = acos(max(-1, min(1, cos_angle)));
    
    % WUL orbit range error (maximum over all user locations)
    if angle < beta || angle > (pi - beta)
        Dr_wul = Dr_mag;  % Worst case: error points toward/away from Earth
    else
        Dr_wul = Dr_mag * abs(sin(angle));  % Projected at tangent point
    end
    
    % WUL SISRE = sqrt(Dr_wul^2 + clock_error^2)
    SISRE_wul(i) = sqrt(Dr_wul^2 + dclk_m(i)^2);
end

SISRE_wul_rms = sqrt(mean(SISRE_wul.^2));
fprintf('  WUL SISRE (RMS): %.4f m\n', SISRE_wul_rms);
fprintf('  Reference (Montenbruck 2015): 0.82 m (GPS global average)\n\n');

%% ---- STEP 9: DUAL FREQUENCY IONOSPHERE-FREE CHECK ----
% Show how IF combination removes ionosphere

fprintf('DUAL-FREQUENCY IONOSPHERE-FREE ANALYSIS\n');
fprintf('  IF combination: P_IF = (f1^2*P1 - f2^2*P2)/(f1^2-f2^2)\n');
fprintf('  Noise amplification factor: %.3f\n', sqrt(alpha_IF^2 + beta_IF^2));
fprintf('\n  %-5s %10s %10s %10s %12s\n', ...
        'Sat', 'PR_L1(m)', 'PR_L2(m)', 'PR_IF(m)', 'Iono removed(m)');
fprintf('  %s\n', repmat('-',1,55));
for i = 1:n_sat
    fprintf('  G%-4d %10.3f %10.3f %10.3f %12.3f\n', ...
        i, PR_L1(i), PR_L2(i), PR_IF(i), I_L1(i));
end
fprintf('\n');

%% ---- STEP 10: PLOTS ----

figure('Name', 'GPS Pseudorange and SISRE Analysis', ...
       'Position', [50, 50, 1400, 900]);

% --- Plot 1: Pseudorange error by satellite ---
subplot(2,3,1);
bar_data = [orbit_LOS, dclk_m, T_delay, I_L1, M, epsilon];
colors   = [0.2,0.5,0.9; 0.9,0.3,0.3; 0.3,0.8,0.3; ...
            0.9,0.7,0.2; 0.7,0.4,0.9; 0.5,0.5,0.5];
b = bar(1:n_sat, bar_data, 'stacked');
for k = 1:6, b(k).FaceColor = colors(k,:); end
legend({'Orbit','Sat Clock','Tropo','Iono','Multipath','Noise'}, ...
       'Location','best','FontSize',7);
xlabel('Satellite (G1-G6)'); ylabel('Error (m)');
title('Pseudorange Error Budget per Satellite');
grid on; hold on;
plot(1:n_sat, PR_error, 'k^', 'MarkerSize', 8, 'LineWidth', 2);
legend({'Orbit','Sat Clock','Tropo','Iono','Multipath','Noise','Total Error'}, ...
       'Location','best','FontSize',7);

% --- Plot 2: SISRE components ---
subplot(2,3,2);
sisre_components = [wR*abs(dR), wAC*abs(dA), wAC*abs(dC), abs(dclk_m)];
b2 = bar(1:n_sat, sisre_components, 'grouped');
b2(1).FaceColor=[0.9,0.2,0.2]; % Radial — dominant
b2(2).FaceColor=[0.2,0.7,0.9]; % Along
b2(3).FaceColor=[0.2,0.9,0.5]; % Cross
b2(4).FaceColor=[0.9,0.6,0.1]; % Clock
hold on;
plot(1:n_sat, SISRE_full, 'kp', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'k');
legend({'wR|dR|','wAC|dA|','wAC|dC|','|dClk|','SISRE_{full}'}, ...
       'Location','best','FontSize',8);
xlabel('Satellite (G1-G6)'); ylabel('Error Magnitude (m)');
title('SISRE Components per Satellite');
grid on;

% --- Plot 3: Elevation vs SISRE and PR error ---
subplot(2,3,3);
yyaxis left;
plot(el_deg, SISRE_full, 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'r');
ylabel('SISRE (m)', 'Color', 'r');
yyaxis right;
plot(el_deg, abs(PR_error), 'b^', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'b');
ylabel('|Pseudorange Error| (m)', 'Color', 'b');
xlabel('Satellite Elevation (degrees)');
title('Elevation vs SISRE and PR Error');
grid on;
for i = 1:n_sat
    text(el_deg(i)+0.5, SISRE_full(i), sprintf('G%d',i), 'FontSize', 8);
end

% --- Plot 4: Orbit errors in RAC frame ---
subplot(2,3,4);
hold on; grid on;
bar_rac = [dR, dA, dC];
b3 = bar(1:n_sat, bar_rac, 'grouped');
b3(1).FaceColor=[0.9,0.2,0.2];
b3(2).FaceColor=[0.2,0.6,0.9];
b3(3).FaceColor=[0.2,0.9,0.4];
yline(sigma_R,  'r--', sprintf('\\sigma_R=%.2fm',sigma_R),  'LabelHorizontalAlignment','left');
yline(-sigma_R, 'r--');
yline(sigma_A,  'b--', sprintf('\\sigma_A=%.2fm',sigma_A),  'LabelHorizontalAlignment','left');
yline(-sigma_A, 'b--');
legend({'Radial (dR)','Along (dA)','Cross (dC)'}, 'Location','best');
xlabel('Satellite (G1-G6)'); ylabel('Orbit Error (m)');
title('Broadcast Orbit Errors in RAC Frame');

% --- Plot 5: Ionospheric delay by elevation ---
subplot(2,3,5);
hold on; grid on;
el_smooth = linspace(10,90,100) * pi/180;
I_smooth  = zeros(100,1);
for k = 1:100
    I_vert = 40.3e16 * TEC / f_L1^2;
    F_k    = 1/sqrt(1-(R_earth*cos(el_smooth(k))/(R_earth+350e3))^2);
    I_smooth(k) = I_vert * F_k;
end
plot(el_smooth*180/pi, I_smooth, 'b-', 'LineWidth', 2);
scatter(el_deg, I_L1, 80, 'r', 'filled');
for i=1:n_sat
    text(el_deg(i)+1, I_L1(i), sprintf('G%d',i), 'FontSize', 8);
end
xlabel('Elevation Angle (degrees)');
ylabel('Ionospheric Delay L1 (m)');
title(sprintf('Ionospheric Delay (TEC=%d TECU, Chennai)', TEC));
legend('Model curve', 'Satellites', 'Location','northeast');

% --- Plot 6: SISRE comparison bar ---
subplot(2,3,6);
hold on; grid on;

% Compare per-satellite SISRE values
x = 1:n_sat;
bar(x-0.25, SISRE_orb,  0.25, 'FaceColor',[0.4,0.7,1.0], 'FaceAlpha',0.8);
bar(x,      SISRE_full, 0.25, 'FaceColor',[0.9,0.3,0.3], 'FaceAlpha',0.8);
bar(x+0.25, abs(SISRE_proj), 0.25, 'FaceColor',[0.3,0.8,0.4], 'FaceAlpha',0.8);

% Draw reference lines from Montenbruck 2015
yline(0.71, 'k--', 'GPS avg 0.71m (Montenbruck 2015)', ...
      'LabelHorizontalAlignment','left', 'LineWidth', 1.5);
yline(SISRE_full_rms, 'r:', sprintf('This sim: %.3fm', SISRE_full_rms), ...
      'LabelHorizontalAlignment','right', 'LineWidth', 1.5);

legend({'SISRE_{orb}','SISRE_{full}','SISRE_{projected}'}, ...
       'Location','northeast');
xlabel('Satellite (G1-G6)'); ylabel('SISRE (m)');
title('SISRE Comparison per Satellite');
set(gca, 'XTick', 1:n_sat, 'XTickLabel', arrayfun(@(x) sprintf('G%d',x), 1:n_sat, 'UniformOutput', false));

sgtitle('GPS Pseudorange and SISRE Analysis', ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ---- FINAL SUMMARY TABLE ----
fprintf('==============================================\n');
fprintf('           FINAL RESULTS SUMMARY             \n');
fprintf('==============================================\n');
fprintf('  Satellites simulated:     %d GPS\n', n_sat);
fprintf('  Receiver location:        Chennai (%.1fN, %.1fE)\n', ...
        lat_r*180/pi, lon_r*180/pi);
fprintf('  Receiver clock offset:    %.3f ms\n', dt_r*1000);
fprintf('\n');
fprintf('  PSEUDORANGE (L1):\n');
fprintf('    Mean error:             %.4f m\n', mean(PR_error));
fprintf('    RMS error:              %.4f m\n', sqrt(mean(PR_error.^2)));
fprintf('    Max error:              %.4f m\n', max(abs(PR_error)));
fprintf('\n');
fprintf('  SISRE:\n');
fprintf('    Orbit-only RMS:         %.4f m\n', SISRE_orb_rms);
fprintf('    Full (orb+clk) RMS:     %.4f m\n', SISRE_full_rms);
fprintf('    WUL SISRE:              %.4f m\n', SISRE_wul_rms);
fprintf('    Reference (2015):       0.71 m (Montenbruck Table 5)\n');
fprintf('\n');
fprintf('  ERROR BUDGET (dominant terms):\n');
fprintf('    Ionosphere (L1):        %.2f m RMS\n', sqrt(mean(I_L1.^2)));
fprintf('    Troposphere:            %.2f m RMS\n', sqrt(mean(T_delay.^2)));
fprintf('    Orbit errors (LOS):     %.4f m RMS\n', sqrt(mean(orbit_LOS.^2)));
fprintf('    Satellite clock:        %.4f m RMS\n', sqrt(mean(dclk_m.^2)));
fprintf('    Multipath:              %.4f m RMS\n', sqrt(mean(M.^2)));
fprintf('==============================================\n');