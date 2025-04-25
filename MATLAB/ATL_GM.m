
Vdc=400;
V_nom = 120*sqrt(2);
omega_nom=2*pi*60;
S_nom=5000;
mp= (2*pi*0.5)/S_nom;
mq= 0.05*V_nom/S_nom;
Ts = 1e-06;

%%% Inverter Filter %%%
% Lf=125e-6*400;
% Rf= 53e-3;
% Cf=3.5e-6*400;
% Rd=5e-6;
% Lg= 0.0012;
% Rg=0.6;

Lf=125e-6;
Rf= 53e-3;
Cf=3.5e-6*300;
Rd=5e-6;
Lg= 0.0012;
Rg=0.6;


%%% Control Parameters %%%%
% BW_voltage=2*pi*400;
% BW_current=2*pi*2000;
% Kpv=BW_voltage*Cf;
% Kiv=BW_voltage*0.1;
% Kpi=BW_current*Lf;
% Kii=BW_current*Rf;
Kpv=0.1;
Kiv=100;
Kpi=30;
Kii=200;