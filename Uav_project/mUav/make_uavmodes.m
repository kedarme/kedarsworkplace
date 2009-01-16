function []=make_uavmodes(force)
%function make_uavmodes
%
% This will regenerate the stateflow chart by taking a template for a given
% vehicle and copying it NUM_AC times.  It will also deal with parsing the
% events correctly such that the local event names do not need to be
% changed.
%
% The first thing this function does is check the number of vehicles that
% currently exist in the stateflow chart.  If that is the correct number
% it does nothing.  Calling this function with a 'force' flag will force
% the stateflow chart to be reconstructed.

global num_ac

if nargin > 1
    error('MAKE_UAVMODES can only be called with a ''force'' flag input');
elseif (nargin == 1)
    if (~ischar(force)) | (~strcmp(force,'force'))
      error('input must be the flag ''force''');
    end
end

disp('updating Stateflow chart...');

rt = sfroot;

chart = rt.find('-isa', 'Stateflow.Chart', '-and', 'Name','uav_modes');

% check to see how many aircraft are in the stateflow machine
if((length(chart.find('-isa','Stateflow.State','-depth', 1))~=(num_ac+1))|nargin==1) % if the correct number , don't do anything
    
    objs = chart.find('-isa','Stateflow.State','-depth', 1);
    for i=1:length(objs)
        objs(i).delete;
    end
    objs = chart.find('-isa','Stateflow.Event');
    for i=1:length(objs)
        objs(i).delete;
    end
    objs = chart.find('-isa','Stateflow.Data');
    for i=1:length(objs)
        objs(i).delete;
    end

    % coordinate system definitions within the stateflow chart
    % this will be held in a cell array
    % The 1,1 element is an N x 2 matrix of N coordinate systems
    % The 1,2 element is an N x 1 cell array of strings which describe each coordinate system
    cs{1,1}  = [0 0];                   % define origin for root coordinate system
    cs{1,2} = 'root coordinate system';
    
    % define CS of parse_events state relative to root
    delx = 15; dely=15;
    ind=strmatch('root coordinate system',cs{1,2});
    cs{1,1} = [cs{1,1};cs{1,1}(ind,1)+delx cs{1,1}(ind,2)+dely]; % define CS of parse_events superstate
    cs{1,2} = cellstr(strvcat(char(cs{1,2}),'parse_events coor sys'));
    
    % define CS of aircraft 1 relative to root cs 
    delx = 15; dely=15;
    ind=strmatch('root coordinate system',cs{1,2});
    cs{1,1} = [cs{1,1};cs{1,1}(ind,1)+delx cs{1,1}(ind,2)+dely]; % define CS of parse_events superstate
    cs{1,2} = cellstr(strvcat(char(cs{1,2}),'cs for aircraft 1'));
    
    %
    % copying from a template file.
    %
    
    load_system('sf_uavmode_template');
    rt=sfroot;
    m = rt.find('-isa', 'Stateflow.Machine', '-and', 'Name','sf_uavmode_template');
    chart_tpl=m.find('-isa','Stateflow.Chart');
    state_tpl=chart_tpl.find('-isa','Stateflow.State','-depth', 1);
    dum = state_tpl.Position;
    width = dum(3); height = dum(4);
    cb = sfclipboard;
    cb.copy(state_tpl);
    close_system('sf_uavmode_template');
    
    % get all State handles before paste
    state_h = chart.find('-isa','Stateflow.State','-depth', 1);
    if ~isempty(state_h)
        state_id_before = [state_h.get.Id];
    else
        state_id_before = [];
    end
    % first aircraft
    ind=strmatch('cs for aircraft 1',cs{1,2});
    cb.pasteTo(chart);  % this did not return the pasted objects as doc stated
    % get all state ids after paste
    state_h = chart.find('-isa','Stateflow.State','-depth', 1);
    state_id_after = [state_h.get.Id];
    % perform a setdiff and find the id of the element just pasted
    state_id = setdiff(state_id_after, state_id_before);
    uav_n = rt.find('Id',state_id);
    
    uav_n.name = ['uav1'];
    uav_n.Position = [cs{1,1}(ind,:) width height];
    % uav_n.Type = 'AND';
    
    sep=15;
    
    for i=2:num_ac
        % get all State handles before paste
        state_h = chart.find('-isa','Stateflow.State','-depth', 1);
        state_id_before = [state_h.get.Id];
        % first aircraft
        cb.pasteTo(chart);  % this did not return the pasted objects as doc stated
        % get all state ids after paste
        state_h = chart.find('-isa','Stateflow.State','-depth', 1);
        state_id_after = [state_h.get.Id];
        % perform a setdiff and find the id of the element just pasted
        state_id = setdiff(state_id_after, state_id_before);
        uav_n = rt.find('Id',state_id);
        
        uav_n.name = ['uav' num2str(i)];
        ind=strmatch(['cs for aircraft ' num2str(i-1)],cs{1,2});
        cs{1,1} = [cs{1,1};[cs{1,1}(ind,1) cs{1,1}(ind,2)+height+sep]];
        cs{1,2} = cellstr(strvcat(char(cs{1,2}),['cs for aircraft ' num2str(i)]));
        uav_n.Position = [cs{1,1}(ind+1,:) width height];
        
        uav_n.isGrouped = 0; % so I can update the label text in substates
        substates_h = uav_n.find('-isa','Stateflow.State','-depth', 1, '-not','Id',uav_n.Id); % find all instincts states
        for j=1:4 % loop on number of instincts
            temp = substates_h(j).label;
            % this is putting the correct row index to matrix
            % also checked that the ordering of the state handles is as the states appear
            % I could match name if I needed to as well, but I don't.
            ind = findstr(temp,'1');  % find all 1s and replace first one
            % had to do this for when 2 digits (or more?!?) replace 1 digit
            temp=[temp(1:(ind(1)-1)) num2str(i) temp((ind(1)+1):length(temp))];
            substates_h(j).label = temp;
        end      
        uav_n.isGrouped = 1; % regroup superstate      
    end
    
    %
    % put in Parse_events state
    %
    parse_h = Stateflow.State(chart);
    parse_h.isSubchart = 1;
    height_uav = height;
    width = 90;  height = 40; sep = 15;
    ind=strmatch(['cs for aircraft ' num2str(num_ac)],cs{1,2});
    parse_h.position = [cs{1,1}(ind,1)  cs{1,1}(ind,2)+height_uav+sep width height];
    label = 'parse_events';
    
    for i=1:num_ac
        label = [label 10 ...
                'on e' num2str(1+4*(i-1)) ': uav' num2str(i) '.aa;' 10 ...
                'on e' num2str(2+4*(i-1)) ': uav' num2str(i) '.oa;' 10 ...
                'on e' num2str(3+4*(i-1)) ': uav' num2str(i) '.ta;' 10 ...
                'on e' num2str(4+4*(i-1)) ': uav' num2str(i) '.fk;' 10];
    end
    
    parse_h.label = label;
    
    %
    % define all external events
    %
    for i=1:num_ac*4
        e_h = Stateflow.Event(chart);
        e_h.Name = ['e' num2str(i)];
        e_h.Scope = 'INPUT_EVENT';
        e_h.Trigger = 'RISING_EDGE_EVENT';
    end
    
    %
    % Add counter of modes for each vehicle
    %
    data_h = Stateflow.Data(chart);
    data_h.Name = 'mode_count'; % this is the name in the template
    data_h.Scope = 'LOCAL_DATA';
    data_h.SaveToWorkspace = 1; % dump it to the workspace at simulation end
    data_h.Props.Array.Size=['[' num2str(num_ac) ' 4]']; % hardcode 4 instincts
    data_h.Props.Array.FirstIndex = '1'; 
    
    %
    % Reconnect the block
    %
    h_uavmode = find_system('vehicles','BlockType','SubSystem','Name','uav_modes');
    mypath=get_param(char(h_uavmode),'parent');
    try % if line exists delete it
      delete_line(mypath,get_param([mypath '/Reshape'],'outputports'));
    end
    add_line(mypath,'Reshape/1', 'uav_modes/Trigger');
end
%
% open and view the final chart
%
% chart.view
