function varargout = stateic(varargin)
% STATEIC Application M-file for stateic.fig
%    FIG = STATEIC launch stateic GUI.
%    STATEIC('callback_name', ...) invoke the named callback.

% Last Modified by GUIDE v2.0 01-Jun-2002 22:02:46

if nargin == 0  % LAUNCH GUI

	fig = openfig(mfilename,'reuse');

	% Use system color scheme for figure:
	set(fig,'Color',get(0,'defaultUicontrolBackgroundColor'));

	% Generate a structure of handles to pass to callbacks, and store it. 
	handles = guihandles(fig);
	guidata(fig, handles);

    % augment handles structure to include certain workspace variables
    %  among other things
    handles.state_ic = evalin('base','state_ic');

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
function varargout = edit_deltax_ic_Callback(h, eventdata, handles, varargin)

handles.state_ic.deltax_ic = str2num(get(h,'string'));
assignin('base','state_ic',handles.state_ic);
guidata(gcbo,handles);

% --------------------------------------------------------------------
function varargout = edit_deltay_ic_Callback(h, eventdata, handles, varargin)

handles.state_ic.deltay_ic = str2num(get(h,'string'));
assignin('base','state_ic',handles.state_ic);
guidata(gcbo,handles);

% --------------------------------------------------------------------
function varargout = edit_deltah_ic_Callback(h, eventdata, handles, varargin)

handles.state_ic.deltah_ic = str2num(get(h,'string'));
assignin('base','state_ic',handles.state_ic);
guidata(gcbo,handles);

% --------------------------------------------------------------------
function varargout = edit_h_ic_Callback(h, eventdata, handles, varargin)

handles.state_ic.h_ic = str2num(get(h,'string'));
assignin('base','state_ic',handles.state_ic);
guidata(gcbo,handles);

% --------------------------------------------------------------------
function varargout = edit_biasy_ic_Callback(h, eventdata, handles, varargin)

handles.state_ic.biasy_ic = str2num(get(h,'string'));
assignin('base','state_ic',handles.state_ic);
guidata(gcbo,handles);

% --------------------------------------------------------------------
function varargout = push_apply_Callback(h, eventdata, handles, varargin)

delete(gcbf)

% --------------------------------------------------------------------
function fillin_gui(handles)

% populates the deltax_ic field
set(handles.edit_deltax_ic,'string',handles.state_ic.deltax_ic);

% populates the deltay_ic field
set(handles.edit_deltay_ic,'string',handles.state_ic.deltay_ic);

% populates the deltah_ic field
set(handles.edit_deltah_ic,'string',handles.state_ic.deltah_ic);

% populates the h_ic field
set(handles.edit_h_ic,'string',handles.state_ic.h_ic);

% populates the biasy_ic field
set(handles.edit_biasy_ic,'string',handles.state_ic.biasy_ic);

