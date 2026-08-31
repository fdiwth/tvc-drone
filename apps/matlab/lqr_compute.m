clear; clc;

% Physical parameters
mass      = 0.405;
g         = 9.81;
thrust_n  = mass * g;
arm       = 0.205;
I_xx      = 0.01802468421;
I_yy      = 0.01802468421;
B_rz_aero = 0.392;             % rad/s^2 per kg of thrust difference

B_rx = (thrust_n * arm) / I_xx;
B_ry = (thrust_n * arm) / I_yy;
B_pz = 1.0 / mass;

dt = 0.01;   % 100 Hz

% States: [rx, ry, rz, pz, drx, dry, drz, dpz, irx, iry, irz, ipz]
A_c = [
    0 0 0 0   1 0 0 0   0 0 0 0 ;   % d(rx)/dt  = drx
    0 0 0 0   0 1 0 0   0 0 0 0 ;   % d(ry)/dt  = dry
    0 0 0 0   0 0 1 0   0 0 0 0 ;   % d(rz)/dt  = drz
    0 0 0 0   0 0 0 1   0 0 0 0 ;   % d(pz)/dt  = dpz
    0 0 0 0   0 0 0 0   0 0 0 0 ;   % d(drx)/dt = (none)
    0 0 0 0   0 0 0 0   0 0 0 0 ;   % d(dry)/dt = (none)
    0 0 0 0   0 0 0 0   0 0 0 0 ;   % d(drz)/dt = (none)
    0 0 0 0   0 0 0 0   0 0 0 0 ;   % d(dpz)/dt = (none)
    1 0 0 0   0 0 0 0   0 0 0 0 ;   % d(irx)/dt = rx
    0 1 0 0   0 0 0 0   0 0 0 0 ;   % d(iry)/dt = ry
    0 0 1 0   0 0 0 0   0 0 0 0 ;   % d(irz)/dt = rz
    0 0 0 1   0 0 0 0   0 0 0 0 ;   % d(ipz)/dt = pz
];

% [servo x, servo y, difference, thrust]
B_c = [
    0,       0,      0,          0    ;   % rx
    0,       0,      0,          0    ;   % ry
    0,       0,      0,          0    ;   % rz
    0,       0,      0,          0    ;   % pz
   -B_rx,    0,      0,          0    ;   % drx
    0,       B_ry,   0,          0    ;   % dry
    0,       0,      B_rz_aero,  0    ;   % drz
    0,       0,      0,          B_pz ;   % dpz
    0,       0,      0,          0    ;   % irx
    0,       0,      0,          0    ;   % iry
    0,       0,      0,          0    ;   % irz
    0,       0,      0,          0    ;   % ipz
];

sys_c = ss(A_c, B_c, eye(12), zeros(12,4));
sys_d = c2d(sys_c, dt, 'zoh');
A_d = sys_d.A;
B_d = sys_d.B;

% Controllability check
Wc = ctrb(A_d, B_d);
rank_Wc = rank(Wc);
fprintf('Controllability rank: %d / %d', rank_Wc, size(A_d,1));
if rank_Wc == size(A_d,1)
    fprintf('  OK\n');
else
    fprintf('  WARNING: system is NOT fully controllable\n');
end

% LQI augmented 12-state DARE
Q12 = diag([ ...
    130.0, ...    % rx
    130.0, ...    % ry
    25.0,  ...    % rz
    15.0,  ...    % pz
    4.0,   ...    % drx
    4.0,   ...    % dry
    0.00001, ...  % drz
    0.1,   ...    % dpz
    10.0,  ...    % irx
    10.0,  ...    % iry
    0.1,   ...    % irz
    0.01   ...    % ipz
]);

R = diag([ ...
    100.0,   ...  % servo_x (rad)
    100.0,   ...  % servo_y (rad)
    17500.0, ...  % differential thrust (kg)
    5.0      ...  % thrust (N)
]);

[K, P12, ~] = dlqr(A_d, B_d, Q12, R);

disp('K integral columns (signs now inherently mapped to plant physics):');
for ch = 1:4
    fprintf('  K(%d,%d) = %.8f\n', ch, 8+ch, K(ch, 8+ch));
end

disp(' ');
disp('const float K[LQR_MAX_INPUTS][LQR_MAX_STATES] = {');
for i = 1:4
    fprintf('    {');
    fprintf('%.8ff, ', K(i,1:11));
    fprintf('%.8ff', K(i,12));
    fprintf('},\n');
end
disp('};');

Acl_d = A_d - B_d * K;
eig_d = eig(Acl_d);
[~, idx] = sort(abs(eig_d), 'descend');
disp(' ');
disp('Closed-loop DISCRETE poles (magnitude must be < 1.0 for stability):');
for e = eig_d(idx).'
    fprintf('  %+.4f%+.4fi (Mag: %.4f)\n', real(e), imag(e), abs(e));
end

u_min = [-0.149066, -0.149066, -0.1,   -0.1*g];
u_max = [ 0.149066,  0.149066,  0.1,    0.05*g];

INTEGRAL_MAX = [1.0, 1.0, 0.2, 0.1];
x_ref12 = zeros(12,1);

A_c_phys = A_c(1:8, 1:8);
B_c_phys = B_c(1:8, :);

x0 = [deg2rad(15); deg2rad(10); deg2rad(20); -0.5; 0; 0; 0; 0];

T = 8.0;
[xs, igs, us, t, tu] = zoh_simulate(x0, zeros(4,1), T, dt, K, x_ref12, ...
    u_min, u_max, INTEGRAL_MAX, A_c_phys, B_c_phys);

% Plot
figure('Position', [100 100 1200 850]);
sgtitle('LQI (12-state) | ZOH @ 100Hz — Yaw: aero-drag torque only');

subplot(4,3,1); plot(t, rad2deg(xs(:,1)), 'b', 'LineWidth', 2); yline(0,'r--'); title('Angle X (deg)'); grid on;
subplot(4,3,2); plot(t, rad2deg(xs(:,2)), 'b', 'LineWidth', 2); yline(0,'r--'); title('Angle Y (deg)'); grid on;
subplot(4,3,3); plot(t, rad2deg(xs(:,3)), 'b', 'LineWidth', 2); yline(0,'r--'); title('Angle Z / Yaw (deg)'); grid on;

subplot(4,3,4); plot(t, rad2deg(xs(:,5)), 'g', 'LineWidth', 2); yline(0,'r--'); title('Rate X (deg/s)'); grid on;
subplot(4,3,5); plot(t, rad2deg(xs(:,6)), 'g', 'LineWidth', 2); yline(0,'r--'); title('Rate Y (deg/s)'); grid on;
subplot(4,3,6); plot(t, rad2deg(xs(:,7)), 'g', 'LineWidth', 2); yline(0,'r--'); title('Rate Z (deg/s)'); grid on;

subplot(4,3,7); plot(t, xs(:,4), 'Color', [0.5 0 0.5], 'LineWidth', 2); yline(0,'r--'); title('Altitude Error (m)'); grid on;
subplot(4,3,8); plot(t, xs(:,8), 'Color', [0.5 0 0.5], 'LineWidth', 2); yline(0,'r--'); title('Altitude Rate (m/s)'); grid on;

subplot(4,3,9);
plot(t, igs(:,1), 'b', 'LineWidth', 2); hold on;
plot(t, igs(:,2), 'r', 'LineWidth', 2);
plot(t, igs(:,3), 'g--', 'LineWidth', 1.5);
title('Integrals (rad\cdot s)'); legend('irx','iry','irz'); grid on; hold off;

subplot(4,3,10);
plot(tu, rad2deg(us(:,1)), 'r', 'LineWidth', 2); hold on;
plot(tu, rad2deg(us(:,2)), 'Color', [1 0.5 0], 'LineWidth', 2);
yline(rad2deg(u_max(1)), 'k:'); yline(rad2deg(u_min(1)), 'k:');
title('Servo Output (deg)'); legend('Servo X','Servo Y'); grid on; hold off;

subplot(4,3,11); plot(tu, us(:,3), 'Color', [0 0 0.5], 'LineWidth', 2); yline(0,'r--'); title('Yaw Torque (Nm)'); grid on;
subplot(4,3,12); plot(tu, us(:,4), 'Color', [0.5 0 0], 'LineWidth', 2); yline(0,'r--'); title('Thrust Delta (N)'); grid on;

% Local functions
function [xs, igs, us, t, tu] = zoh_simulate(x0_8, integ0, T, dt, K, x_ref12, u_min, u_max, INTEGRAL_MAX, A_c_phys, B_c_phys)

    n_steps = round(T / dt);
    x  = x0_8(:);
    ig = integ0(:);

    xs  = zeros(n_steps+1, 8);
    igs = zeros(n_steps+1, 4);
    us  = zeros(n_steps, 4);

    xs(1,:)  = x.';
    igs(1,:) = ig.';

    for k = 1:n_steps
        x12   = [x; ig];
        err12 = x12 - x_ref12;

        u = -K * err12;
        u_clamped = min(max(u, u_min.'), u_max.');

        for ch = 1:4
            saturated = (u(ch) <= u_min(ch)*0.99) || (u(ch) >= u_max(ch)*0.99);
            if ~saturated
                ig(ch) = ig(ch) + err12(ch) * dt;
            end
            ig(ch) = min(max(ig(ch), -INTEGRAL_MAX(ch)), INTEGRAL_MAX(ch));
        end

        us(k,:) = u_clamped.';

        [~, y] = ode45(@(tt, xx) A_c_phys*xx + B_c_phys*u_clamped, [0 dt], x);
        x = y(end,:).';

        xs(k+1,:)  = x.';
        igs(k+1,:) = ig.';
    end

    t  = linspace(0, T, n_steps+1).';
    tu = linspace(0, T-dt, n_steps).';
end