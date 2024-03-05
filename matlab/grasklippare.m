clc
clear all
close all

s = tf('s')

m  = 0.38;      % [kg]  massa for batteri och oversta hyllan med skruvar
M  = 0.97;      % [kg]  massan for resten av robotan
D  = 0.18;      % [m]   avstand mellan hjulaxel och batteriets masscentrum
r  = 0.049;     % [m]   hjulets radie
R = 4;       % [Ohm] inre resistansen for DC-motorn
L = 0.225;
La = 2.75e-6 % [H]   inre induktansen for DC-motorn
Ke = 0.0274;     % [-]   omvandlingsfaktor mellan strom och moment 
Kt = 0.0274;    % [-]   omvandlingsfaktor for DC-motorns mot EMK 
b  = 3.5077e-6;       % friktionsfaktor
J = 3.2284e-6     % Tröghetsmoment för motor och hjul

Ts = 1*10^-3
G = Kt/(s*(Ke*Kt+R*b+s*(R*J+La*b)+s^2*La*J))

volt_to_angle = c2d(G,Ts,'zoh')