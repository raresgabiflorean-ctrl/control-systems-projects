A = [0 1 0 0; 0 -0.182 -2.673 0; 0 0 0 1; 0 0.455 31.18 0];
B = [0; 1.818; 0; -4.545];
C = [1 0 0 0; 0 0 1 0];
D = [0; 0];
 
H  = tf(ss(A,B,C,D));           % H(1)=H1(s)=x/F , H(2)=H2(s)=theta/F
H1 = tf([1.818 1.615e-15 -44.54], [1 0.182 -31.18 -4.459 0]);
H2 = tf([-4.545 1.087e-15],[1 0.182 -31.18 -4.459]);
 
Wc = ctrb(A,B);   
rankWc = rank(Wc);
Wo = obsv(A,C);   
rankWo = rank(Wo);
fprintf('rank(Wc) = %d , rank(Wo) = %d  (sistem de ordin %d)\n', rankWc, rankWo, size(A,1));
 
kp = -57.2; 
ki = -133; 
kd = -6.17; 
Tf_filt = 0.001;
C_theta = pid(kp, ki, kd, Tf_filt);          % regulator PID cu filtru pe derivata
 
H0_cl= minreal(feedback(C_theta*H2,1), 1e-6);   % theta(s)/theta_ref(s)
sys_F_theta= feedback(C_theta, H2);                    % F(s)/theta_ref(s)
 
figure('Name','Bucla interioara - theta');
subplot(2,1,1);
step(H0_cl); 
grid on;
title('Raspuns \theta la treapta pe \theta_{ref}');
xlabel('Timp [s]'); 
ylabel('\theta [rad]');
 
subplot(2,1,2);
step(sys_F_theta); 
grid on;
title('Semnal de comanda F(t) - bucla interioara');
xlabel('Timp [s]'); 
ylabel('F [N]');
 
figure('Name','Marje bucla theta');
margin(C_theta*H2); 
grid on;
[Gm_th, Pm_th, ~, ~] = margin(C_theta*H2);
fprintf('Bucla theta: Gm=%.2f dB, Pm=%.2f grade\n', 20*log10(Gm_th), Pm_th);
 
G = minreal(H1*feedback(C_theta,H2));   % procesul "vazut" de bucla exterioara
kp1 = 0.0118; kd1 = 0.0406; 
Tf1 = 0.001;
C_x = pid(kp1, 0, kd1, Tf1);            % regulator PD (fara integrator - x are deja mod integrator liber)
 
G0= feedback(C_x*G, 1);     % x(s)/x_ref(s)
sys_thref_x= feedback(C_x, G);       % theta_ref(s)/x_ref(s) - semnalul intermediar
 
x_ref_amp = 0.05;   % referinta realista de pozitie [m]
[y_x, t_x]= step(G0);             
y_x_scaled= y_x * x_ref_amp;
[y_thref, t_thref]= step(sys_thref_x);     
y_thref_scaled = y_thref * x_ref_amp;
 
figure('Name','Bucla exterioara - pozitie x');
subplot(2,1,1);
plot(t_x, y_x_scaled); 
grid on;
title(['Raspuns x la treapta pe x_{ref} = ' num2str(x_ref_amp) ' m']);
xlabel('Timp [s]'); 
ylabel('x [m]');
 
subplot(2,1,2);
plot(t_thref, y_thref_scaled); 
grid on;
title('Semnal \theta_{ref}(t) cerut de bucla exterioara');
xlabel('Timp [s]'); 
ylabel('\theta_{ref} [rad]');
 
fprintf('Varf theta_ref necesar: %.4f rad (%.2f grade)\n', ...
    max(abs(y_thref_scaled)), max(abs(y_thref_scaled))*180/pi);
 
figure('Name','Marje bucla x');
margin(C_x*G); 
grid on;
[Gm_x, Pm_x, ~, ~] = margin(C_x*G);
fprintf('Bucla x: Gm=%.2f dB, Pm=%.2f grade\n', 20*log10(Gm_x), Pm_x);
 
Q = diag([4 1 100 1]);     % ponderi Bryson: x<=0.5m, xdot<=1m/s, theta<=0.1rad, thetadot<=1rad/s
R = 0.01;                  % efort de comanda maxim acceptat ~10N
 
P_riccati = icare(A,B,Q,R);          % solutia ecuatiei Riccati
K_lqr = lqr(A,B,Q,R);            % castigul optim
 
Acl_lqr = A - B*K_lqr;
poli_lqr = eig(Acl_lqr);
fprintf('Poli LQR: '); 
disp(poli_lqr');
 
sys_cl_lqr = ss(Acl_lqr, B, eye(4), zeros(4,1));
x0 = [0; 0; 0.1; 0];   % theta0 = 0.1 rad, restul zero
 
figure('Name','LQR - raspuns la conditie initiala');
[y, t] = initial(sys_cl_lqr, x0);
 
subplot(4,1,1); plot(t,y(:,1)); grid on; title('Pozitie carucior (x)'); ylabel('x [m]');
subplot(4,1,2); plot(t,y(:,2)); grid on; title('Viteza carucior (x dot)'); ylabel('$\dot{x}$ [m/s]','Interpreter','latex');
subplot(4,1,3); plot(t,y(:,3)); grid on; title('Unghi pendul (\theta)'); ylabel('\theta [rad]');
subplot(4,1,4); plot(t,y(:,4)); grid on; title('Viteza unghiulara (\theta dot)'); ylabel('$\dot{\theta}$ [rad/s]','Interpreter','latex');
xlabel('Timp [s]');
sgtitle('Raspuns LQR la conditie initiala (\theta_0 = 0.1 rad)');
 
F_t = -(K_lqr*y')';
figure('Name','LQR - semnal de comanda');
plot(t,F_t); grid on;
title('Semnal de comanda LQR: F(t) = -K \cdot q(t)');
xlabel('Timp [s]'); ylabel('F [N]');
fprintf('Varf comanda F (LQR): %.2f N\n', max(abs(F_t)));
 
% Precompensare (feedforward) pentru urmarire referinta de pozitie
Nbar = -1/(C(1,:)*inv(Acl_lqr)*B);
x_ref = 0.05;
sys_cl_lqr_ref = ss(Acl_lqr, B*Nbar, eye(4), zeros(4,1));
 
figure('Name','LQR - raspuns cu feedforward');
step(sys_cl_lqr_ref*x_ref); grid on;
title(['Raspuns LQR (feedforward) la x_{ref} = ' num2str(x_ref) ' m']);
xlabel('Timp [s]'); ylabel('Amplitudine (toate starile)');
legend('x','x dot','\theta','\theta dot');
 
% raspuns la aceeasi conditie initiala 
plant = ss(A,B,C,zeros(2,1));
PIDF_ss = ss(C_theta);
sys_cl_full_pid = feedback(plant, PIDF_ss, 1, 2, -1);   % bucla PID inchisa doar pe theta
 
n_ctrl  = order(PIDF_ss);
n_total = order(sys_cl_full_pid);
fprintf('Stari regulator PID: %d , Stari totale sistem inchis: %d\n', n_ctrl, n_total);
 
x0_full = [x0; zeros(n_total-4,1)];
[y_pid, t_pid] = initial(sys_cl_full_pid, x0_full);
 
figure('Name','Comparatie PID vs LQR - conditie initiala');
subplot(2,1,1);
plot(t, y(:,3), 'b', t_pid, y_pid(:,2), 'r'); 
grid on;
legend('LQR','PID'); 
title('Comparatie \theta - raspuns la conditie initiala');
xlabel('Timp [s]'); 
ylabel('\theta [rad]');
 
subplot(2,1,2);
plot(t, y(:,1), 'b', t_pid, y_pid(:,1), 'r'); 
grid on;
legend('LQR','PID'); 
title('Comparatie x - raspuns la aceeasi conditie initiala');
xlabel('Timp [s]'); 
ylabel('x [m]');
 
%(b) raspuns la referinta de pozitie x_ref
[y_lqr_r, t_lqr_r] = step(sys_cl_lqr_ref);  
y_lqr_r = y_lqr_r * x_ref;
[y_pid_x, t_pid_x]  = step(G0);              
y_pid_x = y_pid_x * x_ref;
[y_pid_th, t_pid_th] = step(sys_thref_x);    
y_pid_th = y_pid_th * x_ref;
 
figure('Name','Comparatie PID vs LQR - referinta x_ref');
subplot(2,1,1);
plot(t_lqr_r, y_lqr_r(:,1), 'b', t_pid_x, y_pid_x, 'r'); 
grid on;
legend('LQR','PID'); 
title(['Comparatie x la x_{ref} = ' num2str(x_ref) ' m']);
xlabel('Timp [s]'); 
ylabel('x [m]');
 
subplot(2,1,2);
plot(t_lqr_r, y_lqr_r(:,3), 'b', t_pid_th, y_pid_th, 'r'); 
grid on;
legend('LQR (\theta real)','PID (\theta_{ref} cerut)');
title('Comparatie excursie unghiulara necesara');
xlabel('Timp [s]'); 
ylabel('\theta [rad]');
 
%(c) tabel comparativ numeric
e_pid = -y_pid(:,2);
F_pid_t = lsim(PIDF_ss, e_pid, t_pid);
 
fprintf('\n=== TABEL COMPARATIV PID vs LQR ===\n');
fprintf('%-30s %12s %12s\n', 'Metrica', 'PID', 'LQR');
fprintf('%-30s %12.2f %12.2f\n', 'Efort comanda max (N)', max(abs(F_pid_t)), max(abs(F_t)));
 
prag = 0.002;
idx_pid = find(abs(y_pid(:,2)) > prag, 1, 'last');
idx_lqr = find(abs(y(:,3))     > prag, 1, 'last');
fprintf('%-30s %12.2f %12.2f\n', 'Timp stabilizare theta (s)', t_pid(idx_pid), t(idx_lqr));
fprintf('%-30s %12.4f %12.4f\n', 'Excursie theta la x_ref (rad)', max(abs(y_pid_th)), max(abs(y_lqr_r(:,3))));
 
M0=0.5; 
m0=0.2; 
l0=0.3; 
D_nom=0.1; 
I_nom=0.006; 
g_nom=9.8;
 
cazuri = {
    'NOMINAL', M0,     m0,     l0
    'M +20%' , M0*1.2, m0,     l0
    'M -20%' , M0*0.8, m0,     l0
    'm +20%' , M0,     m0*1.2, l0
    'm -20%' , M0,     m0*0.8, l0
    'l +20%' , M0,     m0,     l0*1.2
    'l -20%' , M0,     m0,     l0*0.8
};
 
fprintf('%-12s %10s %10s\n', 'Caz', 'LQR stabil', 'PID stabil');
for i = 1:size(cazuri,1)
    nume = cazuri{i,1};
    
    [A_test,B_test] = build_AB(cazuri{i,2}, cazuri{i,3}, cazuri{i,4}, D_nom, I_nom, g_nom);
 
    poli_lqr_test = eig(A_test - B_test*K_lqr);
    stabil_lqr = all(real(poli_lqr_test) < -1e-6);
 
    plant_test = ss(A_test, B_test, C, zeros(2,1));
    sys_cl_test = feedback(plant_test, ss(C_theta), 1, 2, -1);
    poli_pid_test = pole(sys_cl_test);
    poli_relevanti = poli_pid_test(abs(poli_pid_test) > 1e-4);   % ignoram modul liber x (~0)
    stabil_pid = all(real(poli_relevanti) < -1e-6);
 
    fprintf('%-12s %10s %10s\n', nume, string(stabil_lqr), string(stabil_pid));
end
 
t_sim  = 0:0.01:10;
d_mic  = 1;    % N
d_mare = 20;   % N
 
sys_cl_lqr_full = ss(Acl_lqr, B, eye(4), zeros(4,1));
 
figure('Name','Robustete - perturbatii de forta');
for i = 1:2
    if i==1 
        d=d_mic; titlu=' mica (1N)'; 
    else
        d=d_mare; titlu=' mare (20N)'; 
    end
    [y_p, t_p] = impulse(sys_cl_lqr_full*d, t_sim);
    subplot(2,2,i);
    plot(t_p, y_p(:,3)); 
    grid on;
    title(['LQR - perturbatie' titlu]);
    xlabel('Timp [s]'); 
    ylabel('\theta [rad]');
end
 
for i = 1:2
    if i==1 
        d=d_mic; titlu=' mica (1N)'; 
    else
        d=d_mare; titlu=' mare (20N)'; 
    end
    [y_p, t_p] = impulse(sys_cl_full_pid*d, t_sim);
    subplot(2,2,i+2);
    plot(t_p, y_p(:,2)); 
    grid on;
    title(['PID - perturbatie' titlu]);
    xlabel('Timp [s]'); 
    ylabel('\theta [rad]');
end
 
theta0_values = [0.05, 0.1, 0.2, 0.3, 0.5, 0.8];
 
figure('Name','Robustete - conditii initiale crescatoare');
hold on;
for th0 = theta0_values
    x0_th = [0;0;th0;0];
    [y_ic, t_ic] = initial(sys_cl_lqr_full, x0_th);
    plot(t_ic, y_ic(:,3), 'DisplayName', sprintf('\\theta_0=%.2f rad', th0));
end
grid on; legend show;
title('LQR - raspuns la conditii initiale crescatoare');
xlabel('Timp [s]'); 
ylabel('\theta [rad]');
yline(0.2, '--r', 'Limita liniarizare (~0.2 rad)', 'HandleVisibility','off');
yline(-0.2, '--r', 'HandleVisibility','off');
 
Fmax   = 1;     % N - aproape de pragul critic de stabilizare
theta0 = 0.1;   % rad
 
[t1,z1] = ode45(@(t,z) pid_rhs(t,z,A,B,kp,ki,kd,Fmax,false), [0 15], [0;0;theta0;0;0]);  % cu windup
[t2,z2] = ode45(@(t,z) pid_rhs(t,z,A,B,kp,ki,kd,Fmax,true),  [0 15], [0;0;theta0;0;0]);  % anti-windup
 
figure('Name','Robustete - efect windup');
plot(t1,z1(:,3),'r', t2,z2(:,3),'b'); 
grid on;
legend('Cu windup','Anti-windup');
title(sprintf('Efect windup - Fmax = %.1f N', Fmax));
xlabel('Timp [s]'); 
ylabel('\theta [rad]');