%% Cruise Control cu regulator PI
% Simularea raspunsului sistemului la o perturbatie
%% Parametrii vehiculului

m = 1000;      % Masa vehiculului [kg]
D = 50;        % Coeficient de frecare [N*s/m]

%% Parametrii regulatorului PI

kp = 100;      % Castig proportional
ki = 10;       % Castig integral

%% Functia de transfer a vehiculului

H = tf(1,[m D]);

%% Definirea regulatorului PI

G = pid(kp,ki);

%% Functiile de transfer in bucla inchisa

T = feedback(G*H,1);   % Raspunsul sistemului la referinta
H0 = feedback(H,G);    % Raspunsul sistemului la perturbatie

%% Semnalele de simulare

t = 0:0.01:100;        % Timpul de simulare

r = ones(size(t));     % Referinta (viteza dorita)

d = zeros(size(t));    % Perturbatia (aparitia unei pante)
d(t >= 30) = 0.2;

%% Simularea sistemului

y_ref = lsim(T,r,t);       % Raspunsul la referinta
y_dist = lsim(H0,d,t);     % Influenta perturbatiei

% Raspunsul total al sistemului
% Perturbatia reprezinta o forta rezistenta (de exemplu, o panta),
% de aceea efectul ei se scade din raspunsul la referinta.
y = y_ref - y_dist;

%% Afisarea rezultatului

plot(t,y,'LineWidth',2)
grid on

title('Cruise Control cu perturbatii')
xlabel('Timp [s]')
ylabel('Viteza normalizata')