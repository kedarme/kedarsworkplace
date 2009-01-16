function out = initthis(str,num_ac)
% OUT = INITTHIS(STR,NUMAC)
%
% INITTHIS will initialize a given input parameter that is
% defined by STR.  NUMAC represents the number of aircraft
% in a particular formation

global tau_t tau_b tau_n minimum_ac_dist minimum_obs_dist desired_vel
global target_cone vel_ic state_ic
global rho S Cd0 k W


strlist = {'tau','minimum_ac_dist','minimum_obs_dist', 'desired_vel',...
           'targetcone_limit','state_ic'};

if nargin ~= 2
    error('Must have 2 input arguments');
elseif ~ischar(str)
    error('First input must be a string')
elseif ~ismember(str,strlist)
    error('This is not one of the variables that can be initialized')
    strlist
end

switch str
case 'tau' % will initialize time constant array
           % [tau_t tau_b tau_n] x number of aircraft
    out = ones(num_ac,1)*[tau_t tau_b tau_n];
case 'minimum_ac_dist'
    out = ones(num_ac,1)*[minimum_ac_dist];
case 'minimum_obs_dist'
    out = ones(num_ac,1)*[minimum_obs_dist];
case 'desired_vel'
    out = ones(num_ac,1)*[desired_vel];
case 'targetcone_limit'
    out = ones(num_ac,1)*[target_cone];
case 'state_ic'
    % state vector [V gam chi thrust mu loadfactor x y h]
    out = zeros(9,num_ac);
    % initial thrust will offset drag for each aircraft
    Cl = 2*W/(rho*vel_ic^2*S);
    Cd = Cd0+k*Cl^2;
    thrust_ic = 0.5*rho*vel_ic^2*S*Cd;
    % lead aircraft index
    lead_index = ceil(num_ac/2);
    % lead aircraft
    out(:,lead_index) = [vel_ic 0 0 thrust_ic 0 1 0 state_ic.biasy_ic state_ic.h_ic]';
    % to the left of lead
    for i=1:lead_index-1
        out(:,lead_index-i) = ...
              [vel_ic 0 0 thrust_ic 0 1 -i*state_ic.deltax_ic -i*state_ic.deltay_ic+state_ic.biasy_ic state_ic.h_ic-i*state_ic.deltah_ic]';
    end
    % to the right of lead
    for i=1:num_ac-lead_index
        out(:,lead_index+i) = ...
              [vel_ic 0 0 thrust_ic 0 1 -i*state_ic.deltax_ic i*state_ic.deltay_ic+state_ic.biasy_ic state_ic.h_ic-i*state_ic.deltah_ic]';
    end
otherwise
    disp('Not initializing anything, outputing empty matrix');
    out=[];
end

    