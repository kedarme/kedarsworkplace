g=32.2; % gravity ft/s^2

global tau_t tau_b tau_n minimum_ac_dist minimum_obs_dist desired_vel
global target_cone vel_ic
global state_ic
global rho S Cd0 k W
global num_ac targets obstacles


tau_t = 1;  % thrust time constant, sec 
tau_b = .5; % bank angle time constant, sec 
tau_n = .5; % load factor time constant, sec 

rho = .002377; % air density, slug/ft^3
S = 400; % wing area, ft^2
Cd0 = .02; % parasite drag coefficient
k = .1; % induced drag coefficient
W = 32000; % weight, lb

omega_v = .3; % velocity bandwidth, sec
omega_gamma = .2; % gamma bandwidth, sec
omega_chi = .2; % chi bandwidth, sec

Tmax = 25600; % maximum thrust, lb
Clmax = 2.0; % maximum lift coefficient
Clmin = -.5; % minimum lift coefficient
nmax = 7; % maximum load factor

num_ac = 5;  % number of aircraft in collabrative control
num_instincts = 4;  % number of instincts that aircraft are responding to

% Aircraft collision avoidance variables
minimum_ac_dist = 200; % minimum seration distance between aircraft
sigma_a = 100; % decay rate in ft for collision avoidance

% Obstacle collision avoidance variables
obstacles.obs_avoidance_on = 1; % turn obstacle avoidance on/off
% original set
% obstacles.positions = [3000 -200 10000;
%                        4500  800 10000];
% obstacles.radius    = [400;
%                        300];   
obstacles.positions = [3000        -200       10500
                       4500         800       10000
                       3500         500       11000];
obstacles.radius    = [400;
                       300
                       200];   

minimum_obs_dist = 1300; % minimum seperation distance between aircraft and obstacles
sigma_o = 1000; % decay rate in ft for collision avoidance
%desired_vel=[0 400 0]; % desired velocity vector for formation

% Target aquisition/flyby variables
targets.tgt_aquire_on = 0; % turn target aquisition on/off
desired_vel=[400 0 0]; % desired velocity vector for formation
sigma_t1 = 50;   % decay rate in ft/s for desired heading and velocity aquisition
sigma_t2 = 1000; % decay rate in ft/s for target aquisition (vel and direction aquisition)
% original set
% targets.positions = [6000 -1500 10000;
%                      4000  1000 10000];
% targets.radius    = [400;
%                      600];
targets.positions = [6000 -1400 10700;
                     4000  1000 10000];
targets.radius    = [400;
                     600];
target_cone = 30*pi/180; % Target cone limit in radians


% Initial conditions for aircraft
%
% because the number of aircraft is a variable, the initial conditions are
% dictated by certain DELTA variables that separate the aircraft, regardless
% of number
%
% will pick an initial velocity
vel_ic = 400;
% Dr A had two example runs.  They corresponded to the following set of values
%  coordinate system is North East Down.  Altitude is negative down.
%  deltax_ic = 50, deltay_ic = 300, biasy_ic = 0, deltah_ic = 0, target run
%  deltax_ic = 25, deltay_ic = 200, biasy_ic = -1600, deltah_ic = 0, obs run
% will pick initial separation distances in all three axes
state_ic.deltax_ic = 50; % positive will be behind lead
state_ic.deltay_ic = 300; % will stagger this distance to the left and right of lead
state_ic.biasy_ic  =  0;  % lead will be biased by this much, positive is 'east'
% deltax_ic = 25; % positive will be behind lead
% deltay_ic = 200; % will stagger this distance to the left and right of lead
% biasy_ic  =  -1600;
state_ic.deltah_ic = 0; % delta in altitude between aircraft, positive is below lead
state_ic.h_ic = 10000; % initial altitude
% gamma, chi will be set to 0 for now
% thrust, mu, loadfactor will be set to 0 0 1 respectively

