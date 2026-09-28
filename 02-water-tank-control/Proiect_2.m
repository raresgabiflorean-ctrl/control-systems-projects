C1 = 0.02;
C2 = 0.02;
R1 = 40;
R2 = 50;
 
% Modelul in spatiul starilor:
A = [-1/(C1*R1),1/(C1*R1);1/(C2*R1),-1/(C2*R1)-1/(C2*R2)];
B = [1/C1; 0];
C = [0 1];
D = 0;
 
% Functia de transfer a procesului
H = tf(ss(A,B,C,D))
 
% pole(H)          % polii procesului (analiza stabilitate)
% nyquist(H)       % diagrama Nyquist (analiza stabilitate)
% step(H)          % raspuns la treapta, proces in bucla deschisa
% rlocus(H)        % loc radacini, folosit la proiectarea manuala P/I

% pidtune(H,'PI')    % regenerare valori PI, daca H se schimba
% pidtune(H,'PID')   % regenerare valori PID, daca H se schimba
 
K2 = pid(0.0281, 0.0212);                    % Regulator PI
K1 = pid(0.0346, 0.0216, 0.0117, 0.0117/100);% Regulator PID
 
% Functii de transfer in bucla inchisa
H01 = feedback(K1*H,1);   % PID
H02 = feedback(K2*H,1);   % PI
 
figure
step(H01, H02)
legend('PID','PI')
title('Raspuns la referinta (treapta)')
 
info_PI  = stepinfo(H02)
info_PID = stepinfo(H01)
 
e_ss_PI  = 1 - dcgain(H02)   % eroare stationara PI 
e_ss_PID = 1 - dcgain(H01)   % eroare stationara PID 
 
 %SEMNALUL DE COMANDA (debit q cerut de regulator)
K2_tf = tf(K2);
K1_tf = tf(K1);
U_PI  = minreal(K2_tf/(1+K2_tf*H));
U_PID = minreal(K1_tf/(1+K1_tf*H));
 
t_cmd = 0:0.001:12;
[y_PI,  t_PI]  = step(U_PI,  t_cmd);
[y_PID, t_PID] = step(U_PID, t_cmd);
 
figure
plot(t_PI, y_PI, 'b', t_PID, y_PID, 'r')
legend('PI','PID')
title('Semnal de comanda (debit q)')
xlabel('Time (s)')
ylabel('q (variatie debit)')
 
%ROBUSTETE IN FRECVENTA (marje de castig/faza)

figure
margin(K2*H)
title('Marje - PI')
 
figure
margin(K1*H)
title('Marje - PID')

% VERIFICARE LINIARITATE (raspuns la amplitudini diferite) 
t_lin = 0:0.01:20;
 
figure
step(H02*0.05, t_lin)
hold on
step(H02*2, t_lin)
legend('Referinta 0.05','Referinta 2')
title('Verificare liniaritate - PI')
 
figure
step(H01*0.05, t_lin)
hold on
step(H01*2, t_lin)
legend('Referinta 0.05','Referinta 2')
title('Verificare liniaritate - PID')
 
% ROBUSTETE LA VARIATIA PARAMETRILOR (+-20%), regulator PI fix

C1_nom = C1; C2_nom = C2; R1_nom = R1; R2_nom = R2;
 
% Caz +20%
C1_v = 1.2*C1_nom; C2_v = 1.2*C2_nom; R1_v = 1.2*R1_nom; R2_v = 1.2*R2_nom;
A_v  = [-1/(C1_v*R1_v), 1/(C1_v*R1_v); 1/(C2_v*R1_v), -1/(C2_v*R1_v)-1/(C2_v*R2_v)];
B_v  = [1/C1_v; 0];
H_v  = tf(ss(A_v,B_v,[0 1],0));
H_v_PI = feedback(K2*H_v,1);
 
% Caz -20%
C1_v2 = 0.8*C1_nom; C2_v2 = 0.8*C2_nom; R1_v2 = 0.8*R1_nom; R2_v2 = 0.8*R2_nom;
A_v2  = [-1/(C1_v2*R1_v2), 1/(C1_v2*R1_v2); 1/(C2_v2*R1_v2), -1/(C2_v2*R1_v2)-1/(C2_v2*R2_v2)];
B_v2  = [1/C1_v2; 0];
H_v2  = tf(ss(A_v2,B_v2,[0 1],0));
H_v2_PI = feedback(K2*H_v2,1);
 
figure
step(H02, H_v_PI, H_v2_PI)
legend('Nominal','+20%','-20%')
title('Robustete la variatia parametrilor - PI')
 
stepinfo(H_v_PI)
stepinfo(H_v2_PI)
 
% Test gradual (-20% .. +20%), verifica evolutia performantei
procente = [-20 -15 -10 -5 5 10 15 20];
for i = 1:length(procente)
    p = procente(i)/100;
    C1_t = (1+p)*C1_nom; C2_t = (1+p)*C2_nom;
    R1_t = (1+p)*R1_nom; R2_t = (1+p)*R2_nom;
    A_t  = [-1/(C1_t*R1_t), 1/(C1_t*R1_t); 1/(C2_t*R1_t), -1/(C2_t*R1_t)-1/(C2_t*R2_t)];
    B_t  = [1/C1_t; 0];
    H_t  = tf(ss(A_t,B_t,[0 1],0));
    H_t_PI = feedback(K2*H_t,1);
 
    info_t = stepinfo(H_t_PI);
    fprintf('Variatie %+d%%: Overshoot = %.2f%%, SettlingTime = %.2fs\n', ...
        procente(i), info_t.Overshoot, info_t.SettlingTime);
end

 % RASPUNS LA PERTURBATIE DE DEBIT (scurgere/variatie debit)
 
S_PI  = feedback(H,K2);
S_PID = feedback(H,K1);
 
% Perturbatie mica (amplitudine 1)
figure
step(S_PI, S_PID)
legend('PI','PID')
title('Raspuns la perturbatie (variatie debit)')
xlabel('Time (s)')
ylabel('Variatie nivel h_2')
 
% Perturbatie majora (scurgere mare, amplitudine 5, aplicata la t=15s)
t_p = 0:0.01:40;
r   = ones(size(t_p));       % referinta constanta = 1
d   = 5*(t_p >= 15);         % scurgere: apare la t=15s, amplitudine 5
 
y_PI_tot  = lsim(H02, r, t_p) + lsim(S_PI,  d, t_p);
y_PID_tot = lsim(H01, r, t_p) + lsim(S_PID, d, t_p);
 
figure
plot(t_p, y_PI_tot, 'b', t_p, y_PID_tot, 'r')
legend('PI','PID')
title('Raspuns la scurgere majora (debit pierdut = 5)')
xlabel('Time (s)')
ylabel('Nivel h_2')
grid on