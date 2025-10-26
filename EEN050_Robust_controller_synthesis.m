%% Robust and Nonlinear Control, EEN050:
% Template for assignment 1, 2, and 3
%-------------------------------------------------
%-------------------------------------------------
%Initialization
clear all;
close all;
clc
% READ THIS:
% - Run this section first and do not overwrite any of the variables here
% besides (if necessary) input names and output names for the ss-objects.

%Low fidelity F16 longitudinal model from Aircraft Control and Simulation (B.L.Stevens-F.L.Lewis) pp. 156
%The linearized dynamic of the airplane F16
A_n=[-0.127 -235 -32.2 -9.51 0.314;
     -7E-4 -0.969 0 0.908 -2E-4;
      0 0 0 1 0;
      9E-4 -4.56 0 -1.58 0;
      0 0 0 0 -5];
%States
%[V(ft/s) speed, alpha(rad) angle of attack, theta(rad) pitch angle, q(rad/s) pitch rate, T(lb) engine_power]'
B_n=[0 -0.244;
    0 -0.00209; 
    0 0;
    10 -0.199;
    1087 0];
%Control inputs
%[thrust (N); elevator_deflection(rad)]'
C_n=[0 57.3 0 0 0;
     0 0 0 1 0;
     0.0208 15.2 0 1.45 0];
%Measured outputs
%[alpha(deg); q(rad/s); normal_acceleration(ft/s^2) ]
D_n=[0 0;
     0 0;
     0 0.033];
%Note, elevator deflection has a direct effect on vertical/normal
%accelleration
Gn=ss(A_n,B_n,C_n,D_n);
Gn.InputName = 'utilde';
Gn.OutputName = 'y';
% Actuator dynamics (nominal):
GT = tf(1,[1/(2.5*10) (1/2.5+1/10) 1]); % Thrust
Ge = tf(1,[1/25 1]);                    % Elevator deflection
Ga  = ss([GT, 0;0, Ge]);                % Actuator dynamics
Ga.InputName = 'u';
Ga.OutputName = 'ytilde';

% Disturbance model:
dryden =  tf([0.9751 0.2491],[1 0.885 0.1958]);
Wd = ss([0;dryden]);
Wd.InputName = 'd';
Wd.OutputName = 'Wd';

% Noise filter Wn:
wn1 = rad2deg(0.001);
wn2 = 0.001;
ft2m = 0.3048; % feet/meter
wn3 = 0.001/ft2m;
Wn = ss(diag([wn1,wn2,wn3]));
Wn.OutputName = 'Wn';
Wn.InputName = 'n';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LQG design %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Assemble the physical system
G = Gn*[Ga Wd];

[A,B,C,~] = ssdata(G); % Retrive model matrices
% Number of states, control inputs, process noise, measurement noise
nx = length(A); nu = 2; nw = 1; ny = 3;

Glqi = ss(A,B(:,1:2),C(1,:),0); % ss-object is used for the LQI design
% LQI design - weigthing matrices
Q = blkdiag(eye(nx),10000);
R = eye(nu);

% Compute the feedback gain K
[K,~,~] = lqi(Glqi,Q,R);

% Kalman filter design w. 'kalman'
Gkalman = ss(A,B(:,3),C,0);% ss-object for filter design
QN = eye(nw);  % Process noise covariance
RN = eye(ny);  % Measurement noise covariance
[~,L,~] = kalman(Gkalman,QN,RN); % Compute the filter

% Split the gain into the state feedback gain and the integral gain
Kx = K(:,1:end-1);  % state feedback
Ki = K(:,end);      % integral gain
Bu = B(:,1:2);      % this is the part of B that multiplies with u

% Construct the system matrices
Ac = [A-L*C-Bu*Kx -Bu*Ki;-C(1,:) 0];
Bc = [L zeros(nx,1);0 0 0 1];
Cc = -K;
Dc = 0;

% Represent the LQG controller as a dynamical system
% Its input is [ytilde;r]
% Its output is [u]
LQG = ss(Ac,Bc,Cc,Dc);
LQG.InputName={'ytilde(1)','ytilde(2)','ytilde(3)','r'};
LQG.Outputname = {'u(1)','u(2)'};
'You can ignore the above name conflict'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% A1/Ex1 - Open loop analysis
clc
G_F16 = Gn*Ga;                              % Airplane model

% Is the airplane model G_F16 is stable?

% Since the model is linearized, we are checking local stability of the
% system, in this case during level flight.
% For an open loop system it is sufficient to look at the sign of the
% eigenvalues for the system model, all eigenvalues should be strictly
% negative. poles should be in the left half plane.

eig(G_F16.A); % has only strictly negative eigenvalues => Stable.
% pole(G_F16); and pzplot(G_F16); confirms that poles are in the left half plane = Stable.

% Is the airplane model G_F16 minimal order?
% The condition for a minimal order system is that it is both
% 1. Stable, yes, as shown above.
% 2. Observable
rank(obsv(G_F16.A, G_F16.C)) % Is full rank, n=8, so system is observable
% 3. Controllable, or at least reachable.
rank(ctrb(G_F16.A, G_F16.B)); % Is full rank, so system is controllable.
% Therefore the system is of minimal order.

% Plot the singular values of the airplane model G_F16 (see the command
% 'sigma')

figure(1);
grid on;
sigma(G_F16);

% The Hinf norm is just the largest singular value in the entire frequency
% range. This is shown by the upper of the two curves in the plot. The
% largest singular value and therefore the largest system gain appears at w
% ~= 0.149 rad/s

% [sv,wout] = sigma(G_F16);
% m inputs by w amount of frequencies is returned in sv. In this case 2x86

% Compute the H-infinity norm of the airplane model
% (This can be done with the command 'hinfnorm'.)

hinfnorm(G_F16)
% Compute the H-2 norm of the airplane model 
% (The easiest way to do this is with the command 'norm'.)
% example: norm(ss-object of interest,2)

norm(G_F16, 2);

% Compute the H-2 norm of 'Gn'
% It is not possible to calculate the norm of this system because it is not
% a strictly proper. Last term is only proper.
tf(Gn)

%% A1/Ex2 - Uncertain actuator dynamics
% MATLAB-functions to use, 'tf', 'ureal', 'usample', 'step', 'sigma',
% 'blkdiag'.
clc
% Define the uncertainty parameters with 'ureal'
t1   =  ureal('t1',2.5,'percentage',30);
t2   =  ureal('t2',10,'percentage',40);
t3   =  ureal('t3',25,'percentage',25);
gT   =  ureal('gT',1,'percentage',50);
ge   =  ureal('ge',1,'percentage',10);


% Define the uncertain dynamics for thrust
GTu = tf(gT,[1/(t1*t2), (1/t1 + 1/t2), 1]); % Thrust

% Define the uncertain dynamics for elevator deflection
Geu = tf(ge,[1/t3 1]);                    % Elevator deflection

% Define the uncertain actuator dynamics (block diagonal)
Gau = blkdiag(GTu,Geu);
Gau.InputName = 'u';
Gau.OutputName = 'ydelta';

% Create 100 samples of Gau1 and Gau2 with 'usample'
samples = 100; % number of samples
% it takes time to generate 100 samples so feel free to change this
% the value of 'samples' when you are figuring things out.

GTu_samples = usample(GTu, samples);
Geu_samples = usample(Geu, samples);

% Plot the step responses and bode diagrams (sigma plot)
% Feel free to use the following plotting routine:
figure(1)
subplot(2,2,1)
step(GTu_samples)
hold on
step(GTu.NominalValue,'red')
title('Thrust: step response')
legend('random samples','nominal')

subplot(2,2,2)
step(Geu_samples)
hold on
step(Geu.NominalValue,'red')
title('Elevator deflection: step response')
legend('random samples','nominal')

subplot(2,2,3)
sigma(GTu_samples)
hold on
sigma(GTu.NominalValue, 'red')
title('Thrust: singular values')
legend('random samples','nominal')

subplot(2,2,4)
sigma(GTu_samples)
hold on
sigma(GTu.NominalValue, 'red')
title('Elevator deflection: singular values')
legend('random samples','nominal')
hold off

%% Ex2/A2 - (1)
% Simulation of LQG controller w. nominal actuator dynamics
% Consult the documentation for 'lsim' and 'connect' to better understand
% the code
clc
% Simulation parameters:
N = 1000;                  % number of time steps in simulation
T = linspace(0,50,N);      % time vector
flag_noise = 1;            % set to zero to remove noise
flag_x0 = 1;               % set to zero to initialize system at the origin

% Inputs:
r = zeros(N,1);            % reference signal
r(1:200)=-0.5; r(201:400)=1; r(401:600)=5; r(601:800)=-2; r(801:end)=0;
noise = randn(N,4);        % disturbance and measurement noise
U = [r noise*flag_noise];  % input vector

% Define the closed loop system using 'connect':
% Provide appropriate input/output names for each ss-object
Wd.InputName = 'd';
Wd.OutputName = 'Wd';

Ga.InputName = 'u';
Ga.OutputName = 'ydelta';

Wn.InputName = 'n';
Wn.OutputName = 'Wn';

Gn.InputName = 'utilde';
Gn.OutputName = 'y';

LQG.InputName = {'ytilde(1)','ytilde(2)','ytilde(3)','r'};
LQG.OutputName = 'u';

% Summation blocks:
Sum1 = sumblk('utilde = ydelta+Wd',2);
Sum2 = sumblk('ytilde = y+Wn',3);

% Choose inputs and outputs for the resulting closed loop system
inputs = {'r','n','d'};
outputs = {'y(1)'};     % y(1) corresponds with angle of attack [deg]

% Define the closed loop system:
LQG_clp = connect(Ga,Gn,Wd,Wn,LQG,Sum1,Sum2,inputs,outputs);

% Number of states in the closed loop system
nx_LQG = length(LQG_clp.A);
x0 = randn(nx_LQG,1)*flag_x0;   % initial state

% Simulating and plotting
Y_LQG = lsim(LQG_clp,U,T,x0);
figure(2)
plot(T,Y_LQG,T,r,'r--','LineWidth', 1.5)
legend('angle of attack','reference signal')
title('Reference tracking w. LQG on the angle of attack')
ylim([-10, 10])

%% A1/Ex3 - (2)
% Closed loop simulation w. LQG and uncertain actuator dynamics
% Repeat the above simulation with the ss-object 'Ga' replaced by a sample
% of the uncertain ss-object for the actuator dynamics 'Gau'(c.f A1/Ex2).
% The input and output names of the samples actuator dynamics should
% correspond with the input and output names of 'Ga'.

% Take a random sample of the uncertain actuator dynamics

Ga_random = usample(Gau,1);
Ga_random.InputName = 'u';
Ga_random.OutputName = 'ydelta';

% Close the loop with Ga_random instead of Ga
clp_LQGu = connect(Ga_random,Gn,LQG,Wn,Wd,Sum1,Sum2,inputs,outputs);
% simulate and plot
Y_LQGu = lsim(clp_LQGu,U,T,x0);
figure(3)
plot(T,Y_LQGu,T,r,'r--', 'LineWidth', 1.5)
legend('angle of attack','reference signal')
title('Reference tracking w. LQG on the angle of attack (random actuator dynamics)')
ylim([-10, 10])

%% A2/Ex1
clc
% Notes:
% Be mindful of the input/output dimensions when defining the filters.
% Feel free to use 'makeweight' when apropriate
% 
% Wn and Wd are defined in the first code section of this file.
% (Be aware of their input and output names!)
% To define the filter Wm (consult the documentation for 'ucover').
%
% Use 'connect' to contstruct the 'P' matrix.
% For convenience, order the inputs/outputs as follows
% Input: [udelta;r;n;d;u]
% Output: [ydelta;z;v] (v=[y+n;r])
%
% To compute the Hinf controller use 'hinfsyn'
% To compute the H2 controller use 'h2syn'

% Define the filters Wra, We, Wp, Wu

% Wra
kwra = 6.25;
Wra = tf(kwra^2,[1, 2*kwra, kwra^2]);

Wra = [Wra; 0; 0];

% We
We_k =   400; % DC gain
We_wc =  4.3;  % crossover frequency
We_khf = 0.4; % high frequency gain

% See derivation in Filip's notes!

We = paramsToFirstOrderTF(We_k, We_wc, We_khf);
We = [We, 0, 0];

% dcgain(We);
% abs(freqresp(We, We_wc));
% margin(We);

% Wp angle alpha
Wpa_k = 2.5;
Wpa_wc = 0.45;
Wpa_khf = 0.015;
% Wp normal acceleration
Wpan_k = 2.5;
Wpan_wc = 0.7;
Wpan_khf = 0.0063;

% Wpa = tf(makeweight(Wpa_k,Wpa_wc,Wpa_khf))
Wpa = paramsToFirstOrderTF(Wpa_k, Wpa_wc, Wpa_khf); % angle
Wpan = paramsToFirstOrderTF(Wpan_k, Wpan_wc, Wpan_khf); % normal acceleration

Wp = blkdiag(Wpa, 0, Wpan); % See plane table

% Wu
Wu = tf(blkdiag(0, 1/deg2rad(35)));

% Define WmT, Wme, and Wm (read the documentation for 'ucover')
% Feel free to use the samples taken in A1/Ex2

[~, infoWmT] = ucover(GTu_samples, GTu.NominalValue, 2, 'OutputMult'); % Thrust
WmT = infoWmT.W1;

[~, infoWme] = ucover(Geu_samples, Geu.NominalValue, 2, 'OutputMult'); % Elevator deflection
Wme = infoWme.W1;

Wm = blkdiag(WmT,Wme);

% The final weight describes the largest deviation from the nominal value
% at a specific frequency. Output Multiplicative on delta.

% Provide appropriate input/output names

We.InputName = 'we_in';
We.OutputName = 'ze';

Wp.InputName = 'y';
Wp.OutputName = 'zp';

Wn.InputName = 'n';
Wn.OutputName = 'wn_out';

Wra.InputName = 'r';
Wra.OutputName = 'wra_out';

Wu.InputName = 'utilde';
Wu.OutputName = 'zu';

Wm.InputName = 'udelta';
Wm.OutputName = 'wm_out';

Wd.InputName = 'd';
Wd.OutputName = 'wd_out';

Ga.InputName = 'u';
Ga.OutputName = 'ydelta';

Gn.InputName = 'utilde';
Gn.OutputName = 'y';

% Define the summation blocks (there are three):

Sum1 = sumblk('utilde=ydelta+wd_out+wm_out', 2);
Sum2 = sumblk('ytilde=y+wn_out', 3);
Sum3 = sumblk('we_in=wra_out-y', 3);

% Define the appropriate inputs and outputs (the order matters!)
% Input: [udelta;r;n;d;u]
% Output: [ydelta;z;v] (v=[y+n;r])

% Inputs and outputs of general DPK structure

inputs = {'udelta'; 'r'; 'n'; 'd'; 'u'};
outputs = {'ydelta'; 'ze'; 'zp'; 'zu'; 'ytilde'; 'r'}; % (v=[y+n; r]);

P = connect(Ga, Gn, Wra, Wm, We, Wp, Wu, Wn, Wd, Sum1, Sum2, Sum3, inputs, outputs);
size(P); % 7 inputs, 9 outputs, 11 states.

%% A2/Ex2
% Compute the H-infinity controller
nu = 2;
ny = 4;
[HinfCont, C, gamma1] = hinfsyn(P, ny, nu); % r and ytilde length 1 + 3, and u is length 2

% Gamma = 22.5181, NS but not NP.

% plot the singular values
figure(4)
sigma(HinfCont);
title('Singular values of Hinf controller');

%% A2/Ex3
% Compute the H2-controller
[H2Cont, ~, gamma2] = h2syn(P, ny, nu);

% plot the singular values
figure(5)
sigma(H2Cont)
title('Singular values of H2 controller')


%% A3/Ex1
% NS means that the nominal closed loop system N (plant P, controller K) is
% internally stable for all input-output pairs.

% NP means the nominal system has to be NS + N_22 (tf from w to z) has a
% H_inf or i2 norm lower than 1. This means that no disturbances will be
% amplified

% RS, means that the system is NS and N_11 (tf from u_delta to y_delta) has a H_inf
% norm lower than 1. No uncertainty should be amplified.

% Create Nominal closed loop system N using LFT:
K = HinfCont;
N = lft(P,K,nu,ny);

n_udel = 2;
n_ydel = 2;
nw = 5;
nz = 6;

% Make partitions
N11 = N(1:n_ydel, 1:n_udel);
N12 = N(1:n_ydel, n_udel+1:n_udel+nw);
N21 = N(n_ydel+1:n_ydel+nz, 1:n_udel);
N22 = N(n_ydel+1:n_ydel+nz, n_udel+1:n_udel+nw);

% We check if N is stable by looking at eigenvalues, or:
Nstable = isstable(N); % stable => Nominal stability!

% analysis
w = 0.01:0.01:1000; % frequencies of interest

% NP requires NS and hinfnorm(N22) <= 1
N22_norm = hinfnorm(N22) % 22.4793 > 1
% therefore the system is not NP!
figure(6)
sigma(N22, w);
hold on;
yline(0, '--')
title('N22 singular values')

% RS requires NS and hinfnorm(N11) <= 1
N11_norm = hinfnorm(N11); % 1.6838 > 1
% therefore the system is not have RS!
figure(7)
sigma(N11, w);
hold on;
yline(0, '--')
title('N11 singular values')

% RP requires RS and hinfnorm(N) <= 1
N_norm = hinfnorm(N); % 22.5329 > 1
% The system does not have RP since RS is not achieved,
% the hinfnorm is also too large.
figure(8)
sigma(N, w);
hold on;
yline(0, '--')
title('N singular values')


%% A3/Ex2 Simulation
% - Construct with 'connect' the  closed loop system between the LQG, Hinf,
%   and H2 controllers and the open loop plant.
% The open loop plant consists of:
% - Ga (nominal value or a sample of the uncertain actuator dynamics)
% - Wn, Wd, and, Gn
% - The delta block and the filters Wra, We, Wp, Wu, Wm are not included!

% Take a random sample of the uncertain actuator dynamics
Ga_random = usample(Gau,1);

% Define the appropriate summation blocks (there are two)
Sum1 = sumblk('utilde=ydelta+wd_out', 2);
Sum2 = sumblk('ytilde=y+wn_out', 3);

% Choose the apropriate inputs and outputs:
HinfCont.InputName = {'ytilde','ytilde(2)','ytilde(3)', 'r'};
HinfCont.OutputName = 'u';

H2Cont.InputName = {'ytilde(1)','ytilde(2)','ytilde(3)', 'r'};
H2Cont.OutputName = 'u';

inputs = {'r','n','d','u'};
outputs = {'y(1)'};

% Define the closed loop systems:
% LQG closed loop system
LQG_clp = connect(Ga_random,Gn,LQG,Wn,Wd,Sum1,Sum2,inputs,outputs);

% H-infinity closed loop system
Hinf_clp = connect(Ga_random,Gn,HinfCont,Wn,Wd,Sum1,Sum2,inputs,outputs);

% H-2 closed loop system
H2_clp = connect(Ga_random,Gn,H2Cont,Wn,Wd,Sum1,Sum2,inputs,outputs);

% Feel free to use the following plotting routine:
% Simulation parameters:
N = 1000;                  % number of time steps in simulation
T = linspace(0,50,N);      % time vector
flag_noise = 1;            % set to zero to remove noise
flag_x0 = 0;               % set to zero to initialize system at the origin
% Inputs:
r = zeros(N,1);            % reference signal
r(1:200)=-0.5; r(201:400)=1; r(401:600)=5;r(601:800)=-2; r(801:end)=0;
noise = randn(N,4);        % Disturbance and measurement noise
U = [r, noise*flag_noise, zeros(N,2)]; % Input vector

% Initial state(s)
% Number of states is the LQG closed loop system
nx_LQG = length(LQG_clp.A);
% Number of states in the H_(2,infinity) closed loop systems
nx_H = length(Hinf_clp.A);

x0_LQG = randn(nx_LQG,1)*flag_x0;   % initial state (LQG)
x0_H = randn(nx_H,1)*flag_x0;       % initial state (Hinf,H2)

% Simulate:
Yinf =  lsim(Hinf_clp,U,T,x0_H);
Y2   =  lsim(H2_clp,U,T,x0_H);
YLQG =  lsim(LQG_clp,U,T,x0_LQG);

% Feel free to use the following plotting routine.
figure(9)
plot(T,Yinf,T,Y2,T,YLQG,T,r,'r--')
title('Reference tracking on angle of attack [deg]')
legend('Hinf','H2','LQG','reference')
ylim([-3,6])


%% Functions

function TF = paramsToFirstOrderTF(k, wc, khf)

    % Function to calculate first order parameters from:
    % DC gain k
    % high frequency gain khf
    % crossover frequency wc

    p = sqrt( wc^2 * (1-khf^2) / ( (k/khf)^2 * khf^2 - 1) );
    z = k/khf * p;

    TF = tf(khf*[1, z], [1, p]);

end