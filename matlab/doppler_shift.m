clear; clc; close all;

c        = 299792458;
mu       = 3.986005e14;
omega_e  = 7.2921151467e-5;
a_WGS84  = 6378137.0;
e2_WGS84 = 0.00669437999014;
R_earth  = 6371000;
f_L1     = 1575.42e6;

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
toe = 388800;

n_sat   = 4;
sat_pos = zeros(3, n_sat);
sat_vel = zeros(3, n_sat);

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

    for dt_small = [0, 1]
        tk = (toe + dt_small*1) - toe;

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
        phi_k  = vk + omega;

        delta_u = Cus*sin(2*phi_k) + Cuc*cos(2*phi_k);
        delta_r = Crs*sin(2*phi_k) + Crc*cos(2*phi_k);
        delta_i = Cis*sin(2*phi_k) + Cic*cos(2*phi_k);

        uk = phi_k + delta_u;
        rk = A*(1 - e*cos(Ek)) + delta_r;
        ik = i0 + i_dot*tk + delta_i;

        xk_orb = rk * cos(uk);
        yk_orb = rk * sin(uk);
        Omega_k = Omega0 + (Omega_dot - omega_e)*tk - omega_e*toe;

        xs = xk_orb*cos(Omega_k) - yk_orb*cos(ik)*sin(Omega_k);
        ys = xk_orb*sin(Omega_k) + yk_orb*cos(ik)*cos(Omega_k);
        zs = yk_orb*sin(ik);

        if dt_small == 0
            sat_pos(:,i) = [xs; ys; zs];
        else
            sat_vel(:,i) = ([xs;ys;zs] - sat_pos(:,i)) / 1.0;
        end
    end
end

lat_true = 13.08 * pi/180;
lon_true = 80.27 * pi/180;
h_true   = 10.0;
N_true   = a_WGS84 / sqrt(1 - e2_WGS84*sin(lat_true)^2);
X_true   = (N_true + h_true) * cos(lat_true) * cos(lon_true);
Y_true   = (N_true + h_true) * cos(lat_true) * sin(lon_true);
Z_true   = (N_true*(1-e2_WGS84) + h_true) * sin(lat_true);
r_true   = [X_true; Y_true; Z_true];

R_ENU2ECEF = [-sin(lon_true),              cos(lon_true),             0;
              -sin(lat_true)*cos(lon_true), -sin(lat_true)*sin(lon_true), cos(lat_true);
               cos(lat_true)*cos(lon_true),  cos(lat_true)*sin(lon_true), sin(lat_true)];

dt_r_true  = 1.0e-3;
ddt_r_true = 0.1e-6;
sigma_PR   = 3.0;

rho_true   = zeros(n_sat,1);
rho_dot    = zeros(n_sat,1);
fD         = zeros(n_sat,1);
PR_nodop   = zeros(n_sat,1);
PR_withdop = zeros(n_sat,1);

delta_t_proc = 0.02;

for i = 1:n_sat
    diff        = sat_pos(:,i) - r_true;
    rho_true(i) = norm(diff);
    e_LOS       = diff / rho_true(i);

    rho_dot(i) = dot(e_LOS, sat_vel(:,i));

    fD(i) = -(f_L1/c) * (rho_dot(i) + c*ddt_r_true);

    PR_nodop(i)   = rho_true(i) + c*dt_r_true + sigma_PR*randn();

    doppler_bias  = rho_dot(i) * delta_t_proc;
    PR_withdop(i) = rho_true(i) + c*dt_r_true + doppler_bias + sigma_PR*randn();
end

function [pos_est, err_hist] = solve_pos(PR, sat_pos, r_true, c)
    x_est  = [0;0;0];
    dt_est = 0;
    n      = size(sat_pos,2);
    err_hist = [];
    for iter = 1:20
        H = zeros(n,4);
        b = zeros(n,1);
        for i = 1:n
            diff    = sat_pos(:,i) - x_est;
            rho_est = norm(diff);
            b(i)    = PR(i) - (rho_est + c*dt_est);
            H(i,:)  = [-diff(1)/rho_est,-diff(2)/rho_est,-diff(3)/rho_est,1];
        end
        dx     = (H'*H)\(H'*b);
        x_est  = x_est  + dx(1:3);
        dt_est = dt_est + dx(4)/c;
        err_hist(end+1) = norm(x_est - r_true);
        if norm(dx(1:3)) < 1e-6, break; end
    end
    pos_est = x_est;
end

[pos_nodop,   err_nodop]   = solve_pos(PR_nodop,   sat_pos, r_true, c);
[pos_withdop, err_withdop] = solve_pos(PR_withdop, sat_pos, r_true, c);

err_ENU_nodop   = R_ENU2ECEF' * (pos_nodop   - r_true);
err_ENU_withdop = R_ENU2ECEF' * (pos_withdop - r_true);

fprintf('Without Doppler effect:\n');
fprintf('  3D error = %.4f m\n', norm(pos_nodop - r_true));
fprintf('  ENU: E=%.3f  N=%.3f  U=%.3f m\n\n', err_ENU_nodop);

fprintf('With Doppler effect (%.0f ms processing delay):\n', delta_t_proc*1000);
fprintf('  3D error = %.4f m\n', norm(pos_withdop - r_true));
fprintf('  ENU: E=%.3f  N=%.3f  U=%.3f m\n\n', err_ENU_withdop);

fprintf('Doppler shift per satellite:\n');
for i = 1:n_sat
    fprintf('  PRN%d: range_rate=%.2f m/s  fD=%.2f Hz  bias=%.3f m\n', ...
        i, rho_dot(i), fD(i), rho_dot(i)*delta_t_proc);
end

figure('Position',[100,100,1200,900]);

subplot(2,2,1);
hold on; grid on;
bar_x = (1:n_sat) - 0.2;
bar(bar_x, PR_nodop   - (rho_true + c*dt_r_true), 0.35, 'FaceColor',[0.2,0.6,1.0]);
bar_x2 = (1:n_sat) + 0.2;
bar(bar_x2, PR_withdop - (rho_true + c*dt_r_true), 0.35, 'FaceColor',[1.0,0.3,0.3]);
yline(0,'k-','LineWidth',1.5);
set(gca,'XTick',1:n_sat,'XTickLabel',{'PRN1','PRN2','PRN3','PRN4'});
ylabel('Pseudorange Error (m)');
title('PR Error: Without vs With Doppler Effect');
legend('No Doppler','With Doppler','Location','best');

subplot(2,2,2);
hold on; grid on;
plot(1:length(err_nodop),   err_nodop,   'b-o','LineWidth',2,'MarkerFaceColor','b');
plot(1:length(err_withdop), err_withdop, 'r-o','LineWidth',2,'MarkerFaceColor','r');
xlabel('Iteration');
ylabel('3D Position Error (m)');
title('Convergence: Without vs With Doppler');
legend('No Doppler','With Doppler','Location','best');
set(gca,'YScale','log');

subplot(2,2,3);
hold on; grid on;
bar(1:n_sat, rho_dot, 'FaceColor',[0.3,0.7,0.3]);
for i = 1:n_sat
    text(i, rho_dot(i)+sign(rho_dot(i))*5, ...
         sprintf('%.1f Hz\n%.2f m', fD(i), rho_dot(i)*delta_t_proc), ...
         'HorizontalAlignment','center','FontSize',8);
end
set(gca,'XTick',1:n_sat,'XTickLabel',{'PRN1','PRN2','PRN3','PRN4'});
ylabel('Range Rate (m/s)');
title(sprintf('Doppler Range Rate and Bias (\\Deltat=%.0fms)',delta_t_proc*1000));
grid on;
yline(0,'k-','LineWidth',1);

subplot(2,2,4);
hold on; grid on;
components = {'East','North','Up'};
x_pos = [1,2,3];
bar(x_pos - 0.2, err_ENU_nodop,   0.35, 'FaceColor',[0.2,0.6,1.0]);
bar(x_pos + 0.2, err_ENU_withdop, 0.35, 'FaceColor',[1.0,0.3,0.3]);
yline(0,'k-','LineWidth',1.5);
set(gca,'XTick',1:3,'XTickLabel',components);
ylabel('Position Error (m)');
title(sprintf('ENU Error — No Doppler: %.2fm vs With Doppler: %.2fm', ...
    norm(err_ENU_nodop), norm(err_ENU_withdop)));
legend('No Doppler','With Doppler','Location','best');
grid on;

sgtitle('Effect of Doppler Shift on GPS Receiver Position','FontSize',13,'FontWeight','bold');