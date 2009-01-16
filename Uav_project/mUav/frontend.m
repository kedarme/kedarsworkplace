function varargout = frontend(varargin)
% FRONTEND Application M-file for frontend.fig
%    FIG = FRONTEND launch frontend GUI.
%    FRONTEND('callback_name', ...) invoke the named callback.

% Last Modified by GUIDE v2.0 01-Jun-2002 22:21:26

if nargin == 0  % LAUNCH GUI

	fig = openfig(mfilename,'reuse');

	% Use system color scheme for figure:
	set(fig,'Color',get(0,'defaultUicontrolBackgroundColor'));

	% Generate a structure of handles to pass to callbacks, and store it. 
	handles = guihandles(fig);
	guidata(fig, handles);

    % augment handles structure to include certain workspace variables
    %  among other things
    handles.num_ac  = evalin('base','num_ac');
    handles.targets = evalin('base','targets');
    handles.num_targets = length(handles.targets.radius);
    handles.obstacles = evalin('base','obstacles');
    handles.num_obs = length(handles.obstacles.radius);

    guidata(fig, handles); % save the structure back to the figure
    
    % fill in gui
    fillin_gui(handles);
    
    
	if nargout > 0
		varargout{1} = fig;
	end

elseif ischar(varargin{1}) % INVOKE NAMED SUBFUNCTION OR CALLBACK

	try
		if (nargout)
			[varargout{1:nargout}] = feval(varargin{:}); % FEVAL switchyard
		else
			feval(varargin{:}); % FEVAL switchyard
		end
	catch
		disp(lasterr);
	end

end


%| ABOUT CALLBACKS:
%| GUIDE automatically appends subfunction prototypes to this file, and 
%| sets objects' callback properties to call them through the FEVAL 
%| switchyard above. This comment describes that mechanism.
%|
%| Each callback subfunction declaration has the following form:
%| <SUBFUNCTION_NAME>(H, EVENTDATA, HANDLES, VARARGIN)
%|
%| The subfunction name is composed using the object's Tag and the 
%| callback type separated by '_', e.g. 'slider2_Callback',
%| 'figure1_CloseRequestFcn', 'axis1_ButtondownFcn'.
%|
%| H is the callback object's handle (obtained using GCBO).
%|
%| EVENTDATA is empty, but reserved for future use.
%|
%| HANDLES is a structure containing handles of components in GUI using
%| tags as fieldnames, e.g. handles.figure1, handles.slider2. This
%| structure is created at GUI startup using GUIHANDLES and stored in
%| the figure's application data using GUIDATA. A copy of the structure
%| is passed to each callback.  You can store additional information in
%| this structure at GUI startup, and you can change the structure
%| during callbacks.  Call guidata(h, handles) after changing your
%| copy to replace the stored original so that subsequent callbacks see
%| the updates. Type "help guihandles" and "help guidata" for more
%| information.
%|
%| VARARGIN contains any extra arguments you have passed to the
%| callback. Specify the extra arguments by editing the callback
%| property in the inspector. By default, GUIDE sets the property to:
%| <MFILENAME>('<SUBFUNCTION_NAME>', gcbo, [], guidata(gcbo))
%| Add any extra arguments after the last argument, before the final
%| closing parenthesis.


% --------------------------------------------------------------------
function varargout = edit_num_ac_Callback(h, eventdata, handles, varargin)

handles.num_ac = str2double(get(h,'string'));
assignin('base','num_ac',handles.num_ac);
guidata(gcbo,handles);


% --------------------------------------------------------------------
function varargout = push_updatesim_Callback(h, eventdata, handles, varargin)

dothis;


% --------------------------------------------------------------------
function varargout = popup_target_num_Callback(h, eventdata, handles, varargin)

target_num = get(h,'value');
set(handles.edit_target_position,'string',...
    num2str(handles.targets.positions(target_num,:)))
set(handles.edit_target_radius,'string',...
    num2str(handles.targets.radius(target_num)))


% --------------------------------------------------------------------
function varargout = edit_target_position_Callback(h, eventdata, handles, varargin)

target_num = get(handles.popup_target_num,'value');
handles.targets.positions(target_num,:) = str2num(get(h,'string'));
assignin('base','targets',handles.targets);
guidata(gcbo,handles);

% --------------------------------------------------------------------
function varargout = edit_target_radius_Callback(h, eventdata, handles, varargin)

target_num = get(handles.popup_target_num,'value');
handles.targets.radius(target_num) = str2num(get(h,'string'));
assignin('base','targets',handles.targets);
guidata(gcbo,handles);

% --------------------------------------------------------------------
function varargout = popup_obs_num_Callback(h, eventdata, handles, varargin)

obs_num = get(h,'value');
set(handles.edit_obs_position,'string',...
    num2str(handles.obstacles.positions(obs_num,:)))
set(handles.edit_obs_radius,'string',...
    num2str(handles.obstacles.radius(obs_num)))

% --------------------------------------------------------------------
function varargout = edit_obs_position_Callback(h, eventdata, handles, varargin)

obs_num = get(handles.popup_obs_num,'value');
handles.obstacles.positions(obs_num,:) = str2num(get(h,'string'));
assignin('base','obstacles',handles.obstacles);
guidata(gcbo,handles);

% --------------------------------------------------------------------
function varargout = edit_obs_radius_Callback(h, eventdata, handles, varargin)

obs_num = get(handles.popup_obs_num,'value');
handles.obstacles.radius(obs_num) = str2num(get(h,'string'));
assignin('base','obstacles',handles.obstacles);
guidata(gcbo,handles);

% --------------------------------------------------------------------
function varargout = check_target_aquire_on_Callback(h, eventdata, handles, varargin)

handles.targets.tgt_aquire_on = get(h,'value');
assignin('base','targets',handles.targets);
guidata(gcbo,handles);

% --------------------------------------------------------------------
function varargout = check_obs_avoidance_on_Callback(h, eventdata, handles, varargin)

handles.obstacles.obs_avoidance_on = get(h,'value');
assignin('base','obstacles',handles.obstacles);
guidata(gcbo,handles);

% --------------------------------------------------------------------
function varargout = push_delete_target_Callback(h, eventdata, handles, varargin)

% will delete current selected target unless there is only one target in total
if handles.num_targets == 1
    disp('Only one target left, can''t delete it, just turn them off or change this one')
else
    % decriment target number by one
    handles.num_targets = handles.num_targets - 1;
    
    % will delete current selected target
    target_num = get(handles.popup_target_num,'value'); 
    handles.targets.radius(target_num) = [];
    handles.targets.positions(target_num,:) = [];

    % updates the string field of the target popup by one
    set(handles.popup_target_num,'string',...
        [sprintf('%d|',1:handles.num_targets-1) ...
            num2str(handles.num_targets)]); % makes a string 1|2|3...
    
    % change the apporpiate displays to the first target all the time
    set(handles.popup_target_num,'value',1);
    set(handles.edit_target_position,'string',...
        num2str(handles.targets.positions(1,:)))
    set(handles.edit_target_radius,'string',...
        num2str(handles.targets.radius(1)))
    
    % update workspace variables and write handles structure back to GUI
    assignin('base','targets',handles.targets);
    guidata(gcbo,handles);
end

% --------------------------------------------------------------------
function varargout = push_add_target_Callback(h, eventdata, handles, varargin)

% add a target at the end of the targets structure, index I
handles.num_targets = handles.num_targets + 1;
i = handles.num_targets;

%default altitude and radius is of previous target if one exists
if i~=1
    default_alt = handles.targets.positions(i-1,3);
    default_rad = handles.targets.radius(i-1);
else
    default_rad = 0;
    default_rad = 10;
end
handles.targets.positions(i,:) = [0 0 default_alt]; 
handles.targets.radius(i) = default_rad;

% updates the string field of the target popup by one
set(handles.popup_target_num,'string',...
    [sprintf('%d|',1:handles.num_targets-1) ...
     num2str(handles.num_targets)]); % makes a string 1|2|3...

% change the apporpiate displays to show the added target
set(handles.popup_target_num,'value',i);
set(handles.edit_target_position,'string',...
    num2str(handles.targets.positions(i,:)))
set(handles.edit_target_radius,'string',...
    num2str(handles.targets.radius(i)))

% update workspace variables and write handles structure back to GUI
assignin('base','targets',handles.targets);
guidata(gcbo,handles);

% --------------------------------------------------------------------
function varargout = push_add_obs_Callback(h, eventdata, handles, varargin)

% add an obstacle at the end of the obstacles structure, index I
handles.num_obs = handles.num_obs + 1;
i = handles.num_obs;

%default altitude and radius is of previous target if one exists
if i~=1
    default_alt = handles.obstacles.positions(i-1,3);
    default_rad = handles.obstacles.radius(i-1);
else
    default_rad = 0;
    default_rad = 10;
end
handles.obstacles.positions(i,:) = [0 0 default_alt]; 
handles.obstacles.radius(i) = default_rad;

% updates the string field of the obstacles popup by one
set(handles.popup_obs_num,'string',...
    [sprintf('%d|',1:handles.num_obs-1) ...
     num2str(handles.num_obs)]); % makes a string 1|2|3...

% change the appropiate displays to show the added obstacle
set(handles.popup_obs_num,'value',i);
set(handles.edit_obs_position,'string',...
    num2str(handles.obstacles.positions(i,:)))
set(handles.edit_obs_radius,'string',...
    num2str(handles.obstacles.radius(i)))

% update workspace variables and write handles structure back to GUI
assignin('base','obstacles',handles.obstacles);
guidata(gcbo,handles);

% --------------------------------------------------------------------
function varargout = push_delete_obs_Callback(h, eventdata, handles, varargin)

% will delete current selected target unless there is only one target in total
if handles.num_obs == 1
    disp('Only one obstacle left, can''t delete it, just turn them off or change this one')
else
    % decriment target number by one
    handles.num_obs = handles.num_obs - 1;
    
    % will delete current selected target
    obs_num = get(handles.popup_obs_num,'value'); 
    handles.obstacles.radius(obs_num) = [];
    handles.obstacles.positions(obs_num,:) = [];

    % updates the string field of the target popup by one
    set(handles.popup_obs_num,'string',...
        [sprintf('%d|',1:handles.num_obs-1) ...
            num2str(handles.num_obs)]); % makes a string 1|2|3...
    
    % change the apporpiate displays to the first target all the time
    set(handles.popup_obs_num,'value',1);
    set(handles.edit_obs_position,'string',...
        num2str(handles.obstacles.positions(1,:)))
    set(handles.edit_obs_radius,'string',...
        num2str(handles.obstacles.radius(1)))
    
    % update workspace variables and write handles structure back to GUI
    assignin('base','obstacles',handles.obstacles);
    guidata(gcbo,handles);
end

% --------------------------------------------------------------------
function varargout = push_runsim_Callback(h, eventdata, handles, varargin)

set_param('vehicles','simulationcommand','start');

% --------------------------------------------------------------------
function varargout = push_stateic_Callback(h, eventdata, handles, varargin)

stateic; % launch state ic GUI

% --------------------------------------------------------------------
function fillin_gui(handles)

    % populates the num_ac field
    set(handles.edit_num_ac,'string',handles.num_ac);
    
    % populates the target aquisition on field
    set(handles.check_target_aquire_on,'value',...
        handles.targets.tgt_aquire_on);
   
    % populates the target number field
    set(handles.popup_target_num,'string',...
        [sprintf('%d|',1:handles.num_targets-1) ...
         num2str(handles.num_targets)]); % makes a string 1|2|3...
 
    % populates the target position field
    target_num = get(handles.popup_target_num,'value');
    set(handles.edit_target_position,'string',...
        num2str(handles.targets.positions(target_num,:)))
    
    % populates the target radius field
    set(handles.edit_target_radius,'string',...
        num2str(handles.targets.radius(target_num,:)))
    
    % populates the obstacles avoidance on field
    set(handles.check_obs_avoidance_on,'value',...
        handles.obstacles.obs_avoidance_on);

    % populates the obstacle number field
    set(handles.popup_obs_num,'string',...
        [sprintf('%d|',1:handles.num_obs-1) ...
         num2str(handles.num_obs)]); % makes a string 1|2|3...
 
    % populates the obstacle position field
    obs_num = get(handles.popup_obs_num,'value');
    set(handles.edit_obs_position,'string',...
        num2str(handles.obstacles.positions(obs_num,:)))
    
    % populates the obstacle radius field
    set(handles.edit_obs_radius,'string',...
        num2str(handles.obstacles.radius(obs_num,:)))


