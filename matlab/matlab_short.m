clear; clc; close all;

c        = 299792458;
mu       = 3.986005e14;
omega_e  = 7.2921151467e-5;
a_WGS84  = 6378137.0;
e2_WGS84 = 0.00669437999014;

sqrtA_all     = [5153.796875,   5153.648438,   5153.712500,   5153.534375];
e_all         = [0.012182890,   0.008470726,   0.010057926,   0.015032959];
M0_all        = [1.227827499,   0.543872416,  -0.827400219,   2.153971023];
omega_all     = [0.926460011,  -1.124567893,   2.341872109,  -0.782341209];
i0_all        = [0.962716670,   0.958431287,   0.961234567,   0.963218734];
Omega0_all    = [-1.626979270, -0.834512670,   1.234098765,  -2.345671230];
Delta_n_all   = [4.41219e-9,    4.23156e-9,    4.56234e-9,    4.12345e-9];
Omega_dot_all = [-8.17207e-9,  -8.09823e-9,   -8.23145e-9,  -8.11234e-9];
i_dot_all     = [-4.33177e-10, -4.12340e-10,  -4.56123e-10, -4.23456e-10];
Cuc_all = [3.39746e-7,  -2.14577e-7,   1.23456e-7,  -3.45678e-7];
Cus_all = [2.75671e-6,   3.12456e-6,  -1.98765e-6,   2.56789e-6];
Crc_all = [258.375,      245.125,       267.875,       251.500];
Crs_all = [-59.375,      -48.250,       -71.125,       -55.875];
Cic_all = [-1.11759e-7,  -9.87654e-8,   1.23456e-8,  -2.34567e-7];
Cis_all = [-7.45058e-9,   8.34521e-9,  -6.78901e-9,   9.12345e-9];
toe     = 388800;
t       = 388920;

n_sat = 4;
sat_pos = zeros(3, n_sat);

for i = 1:n_sat
    sqrtA     = sqrtA_all(i);
    e         = e_all(i);
    M0        = M0_all(i);
    omega     = omega_all(i);
    i0        = i0_all(i);
    Omega0    = Omega0_all(i);
    Delta_n   = Delta_n_all(i);
    Omega_dot = Omega_dot_all(i);
    i_dot     = i_dot_all(i);
    Cuc = Cuc_all(i); Cus = Cus_all(i);
    Crc = Crc_all(i); Crs = Crs_all(i);
    Cic = Cic_all(i); Cis = Cis_all(i);

    A  = sqrtA^2;
    n0 = sqrt(mu / A^3);
    n  = n0 + Delta_n;

    tk = t - toe;
    if tk >  302400, tk = tk - 604800; end
    if tk < -302400, tk = tk + 604800; end

    Mk = mod(M0 + n*tk, 2*pi);

    Ek = Mk;
    for m = 1:20
        Ek_new = Mk + e*sin(Ek);
        if abs(Ek_new - Ek) < 1e-12, Ek = Ek_new; break; end
        Ek = Ek_new;
    end

    sin_vk = sqrt(1-e^2)*sin(Ek) / (1 - e*cos(Ek));
    cos_vk = (cos(Ek) - e)       / (1 - e*cos(Ek));
    vk     = atan2(sin_vk, cos_vk);

    phi_k   = vk + omega;
    delta_u = Cus*sin(2*phi_k) + Cuc*cos(2*phi_k);
    delta_r = Crs*sin(2*phi_k) + Crc*cos(2*phi_k);
    delta_i = Cis*sin(2*phi_k) + Cic*cos(2*phi_k);

    uk = phi_k + delta_u;
    rk = A*(1 - e*cos(Ek)) + delta_r;
    ik = i0 + i_dot*tk + delta_i;

    xk_orb = rk * cos(uk);
    yk_orb = rk * sin(uk);

    Omega_k = Omega0 + (Omega_dot - omega_e)*tk - omega_e*toe;

    sat_pos(1,i) = xk_orb*cos(Omega_k) - yk_orb*cos(ik)*sin(Omega_k);
    sat_pos(2,i) = xk_orb*sin(Omega_k) + yk_orb*cos(ik)*cos(Omega_k);
    sat_pos(3,i) = yk_orb*sin(ik);
end

fprintf('Satellite ECEF Positions:\n');
for i = 1:n_sat
    fprintf('  PRN %d: X=%12.1f  Y=%12.1f  Z=%12.1f (m)\n', ...
        i, sat_pos(1,i), sat_pos(2,i), sat_pos(3,i));
end

lat_true = 13.08 * pi/180;
lon_true = 80.27 * pi/180;
h_true   = 10.0;
N_true   = a_WGS84 / sqrt(1 - e2_WGS84*sin(lat_true)^2);
X_true   = (N_true + h_true) * cos(lat_true) * cos(lon_true);
Y_true   = (N_true + h_true) * cos(lat_true) * sin(lon_true);
Z_true   = (N_true*(1-e2_WGS84) + h_true) * sin(lat_true);
r_true   = [X_true; Y_true; Z_true];

dt_r_true = 1.0e-3;
sigma_PR  = 3.0;

PR = zeros(n_sat, 1);
for i = 1:n_sat
    rho_true = norm(sat_pos(:,i) - r_true);
    PR(i)    = rho_true + c*dt_r_true + sigma_PR*randn();
end

x_est  = [0; 0; 0];
dt_est = 0;

for iter = 1:20
    H = zeros(n_sat, 4);
    b = zeros(n_sat, 1);

    for i = 1:n_sat
        diff    = sat_pos(:,i) - x_est;
        rho_est = norm(diff);
        b(i)    = PR(i) - (rho_est + c*dt_est);
        H(i,:)  = [-diff(1)/rho_est, -diff(2)/rho_est, -diff(3)/rho_est, 1];
    end

    dx     = (H'*H) \ (H'*b);
    x_est  = x_est  + dx(1:3);
    dt_est = dt_est + dx(4)/c;

    if norm(dx(1:3)) < 1e-6
        break;
    end
end

N_est = a_WGS84 / sqrt(1 - e2_WGS84*sin(atan2(x_est(3), ...
        sqrt(x_est(1)^2+x_est(2)^2)))^2);

lat_est = atan2(x_est(3), sqrt(x_est(1)^2 + x_est(2)^2));
for k = 1:10
    N_est   = a_WGS84 / sqrt(1 - e2_WGS84*sin(lat_est)^2);
    lat_new = atan2(x_est(3) + e2_WGS84*N_est*sin(lat_est), ...
                    sqrt(x_est(1)^2 + x_est(2)^2));
    if abs(lat_new - lat_est) < 1e-12, break; end
    lat_est = lat_new;
end
lon_est = atan2(x_est(2), x_est(1));
h_est   = sqrt(x_est(1)^2+x_est(2)^2)/cos(lat_est) - N_est;

pos_error = norm(x_est - r_true);

fprintf('\nRECEIVER POSITION RESULT\n');
fprintf('  True:      Lat=%.6f  Lon=%.6f  Alt=%.2f m\n', ...
    lat_true*180/pi, lon_true*180/pi, h_true);
fprintf('  Estimated: Lat=%.6f  Lon=%.6f  Alt=%.2f m\n', ...
    lat_est*180/pi, lon_est*180/pi, h_est);
fprintf('  3D Error:  %.4f m\n', pos_error);
fprintf('  Clock offset estimated: %.6f ms\n', dt_est*1000);