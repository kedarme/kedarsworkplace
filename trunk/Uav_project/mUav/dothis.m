% make the appropriate VR world
% get the VR sink handle, assign the correct properties to it
% update that block
make_uavvr('init')

% have to make the stateflow diagram first before the update
% due to timing of the stateflow block and signal propagation
make_uavmodes

% update the diagram after all this stuff is done
%  the stateflow section will work fine
%  the VR area will update the ports on the Multiport Selector
%  and hopefully the ports on the VR Sink
disp('updating diagram...');
set_param('vehicles','SimulationCommand','update');

% now time to reconnect all the appropriate lines since the last update
% put the correct number of ports on the Multiport that will feed the VR world
make_uavvr('connect')

disp('Simulation Updated!');






