function make_uavvr(stage)
%
%

global num_ac targets obstacles

if nargin > 1
    error('MAKE_UAVVR can only be called with a single string input');
elseif (nargin == 1)
    if (~ischar(stage)) | ...
       ~((strcmp(stage,'init')) | ...
       (strcmp(stage,'connect')))
      error('input must be the flag ''connect'' or ''init''');
    end
end

if strcmp(stage,'init')
    
    disp('making VRML world...');
    
    myworld = vrworld('vr_uav.wrl'); % bring up template
    open(myworld);
    NED_h = vrnode(myworld,'VRcoor_to_NED');
    
    scaleFactor = 10;
    
    %
    % Add a UAV node for each UAV
    %   The actual vehicle is input on the URL statement.
    %   Can easily swap out what you want each vehicle to be
    %
    for i=1:num_ac
        transformName = ['uav' num2str(i)];
        
        framePos = [0 0 i];
        frameRot = [0 0 1 0];
        
        newTrans = vrnode(NED_h, 'children', transformName, 'Transform');
        newFrame = vrnode(newTrans, 'children', [transformName, '_vehicle'], 'Inline');
        % can also take the following syntax: setfield(newFrame, 'url', 'myfish.wrl');
        newFrame.url = 'myfish.wrl';
        newTrans.translation = framePos;
        newTrans.rotation = frameRot;
        newTrans.scale =  scaleFactor * ones(1, 3);
        newView = vrnode(newTrans, 'children', [transformName,'_topview'],'Viewpoint');
        newView.description = [transformName, '_top'];
        newView.position = [0 0 -300];
        newView.orientation = [1 0 0 3.1416];
        newView = vrnode(newTrans, 'children', [transformName,'_sideview'],'Viewpoint');
        newView.description = [transformName, '_side'];
        newView.position = [0 300 0];
        newView.orientation = [1 0 0 -1.5708];
        newView = vrnode(newTrans, 'children', [transformName,'_back'],'Viewpoint');
        newView.description = [transformName, '_back'];
        newView.position = [-300 0 0];
        newView.orientation = [-0.5774   -0.5774    0.5774  2.09];
    end
    
    % Add a sphere node for each target if target aquisition is on
    %  make the diffuse color so that one can see through it.
    if (targets.tgt_aquire_on)
        for i=1:length(targets.radius)
            transformName = ['target' num2str(i)];
            newTrans = vrnode(NED_h, 'children', transformName, 'Transform');
            framePos = targets.positions(i,:);
            % change NEU to NED by negating z comp
            framePos(3) = -framePos(3);
            newTrans.translation = framePos;
            newShape = vrnode(newTrans, 'children', [transformName, '_shape'], 'Shape');
            newFrame = vrnode(newShape, 'geometry', [transformName, '_sphere'], 'Sphere');
            newFrame.radius = targets.radius(i);
            newFrame = vrnode(newShape, 'appearance', [transformName, '_appear'], 'Appearance');
            newFrame = vrnode(newFrame, 'material', [transformName, '_mat'], 'Material');
            newFrame.diffuseColor = [0 0.8 0]; % darker green
        end
    end
    % Add a sphere for each obstacle if the obstacles are turned on 
    if (obstacles.obs_avoidance_on)
        for i=1:length(obstacles.radius)
            transformName = ['obstacle' num2str(i)];
            newTrans = vrnode(NED_h, 'children', transformName, 'Transform');
            framePos = obstacles.positions(i,:);
            % change NEU to NED by negating z comp
            framePos(3) = -framePos(3);
            newTrans.translation = framePos;
            newShape = vrnode(newTrans, 'children', [transformName, '_shape'], 'Shape');
            newFrame = vrnode(newShape, 'geometry', [transformName, '_sphere'], 'Sphere');
            newFrame.radius = obstacles.radius(i);
            newFrame = vrnode(newShape, 'appearance', [transformName, '_appear'], 'Appearance');
            newFrame = vrnode(newFrame, 'material', [transformName, '_mat'], 'Material');
            newFrame.diffuseColor = [0.8 0 0]; % darker green
        end
     end
   
    %
    % Save the created world to a new WRL file
    %
    save (myworld, 'temp.wrl');
    
    close(myworld)
    delete(myworld)
    
    %
    % make a simulink diagram with the VRsink
    %
    
    disp('updating VR Sink...');
    
    % find the VR Sink
    h_vrsink=find_system(bdroot('vehicles'),'referenceblock','vrlib/VR Sink');
    h_vrsink=char(h_vrsink);
    
    % extract the inport locations to the existing VR Sink
    %  and delete all lines that exist there currently
    h_vrsubsys = get_param(h_vrsink, 'parent');
    
    h_multi = find_system(h_vrsubsys,'regexp','on','referenceblock','dspindex/Multiport');
    h_multi = char(h_multi);
    
    phandles = get_param(h_vrsink,'porthandles');
    lhandles = get_param(phandles.Inport,'line');
    
    try
        delete_line(cell2mat(lhandles));
    end
        
    phandles = get_param(h_multi,'porthandles');
    lhandles = get_param(phandles.Outport,'line');
    
    try
        delete_line(cell2mat(lhandles));
    end
    
    h_placehldr = find_system(h_vrsubsys,'referenceblock','vrlib/VR Placeholder');
    h_placehldr = char(h_placehldr);

    phandles = get_param(h_placehldr,'porthandles');
    lhandles = get_param(phandles.Outport,'line');
    
    try
        delete_line(cell2mat(lhandles));
    end
    
    % extract it's current position
    pos = get_param(h_vrsink,'position');
    top_left = pos(1:2);
    width = 110;
    height = num_ac*35;
    set_param(h_vrsink,'position',[top_left top_left(1)+width top_left(2)+height]);
    
    %
    % Configure the VR Sink such that the WRL is associated with it
    %  Add the appropriate amount of inports
    %  This is where Sample time will also be indicated

    worldfilename = 'temp.wrl';
    myworld = vrworld(worldfilename);
    open(myworld);
    
    % build strings of the form 'NodesWritten.FieldsWritten#'
    % every UAV node will be having its translation and rotation driven from Simulink
    NodesWritten = [cellstr([char(ones(2*num_ac,1)*'uav') strjust(num2str(floor([1:.5:num_ac num_ac]')),'left')])]';
    FieldsWritten = [cellstr(repmat(strvcat('rotation','translation'),num_ac,1))]';
    temp = strcat(NodesWritten',...
                  repmat('.',2*num_ac,1),...
                  strcat(FieldsWritten',repmat('#',2*num_ac,1)));
    str = cat(2,temp{:}); % deal out and concatenate
    str = str(1:length(str)-1); % drop last '#'
    
    set_param(h_vrsink, 'WorldFileName', worldfilename);
    set_param(h_vrsink, 'FieldsWritten', str);

    while (get(myworld,'opencount')~=1)
        close(myworld);
    end

    
% This will reconnect the VR blocks once the diagram has been updated    
elseif strcmp(stage,'connect')
    
    disp('connecting VR Sink...');
    
    h_vrsink=find_system('vehicles','referenceblock','vrlib/VR Sink');
    h_vrsink=char(h_vrsink);
    
    h_vrsubsys = get_param(h_vrsink, 'parent');
    
    h_multi = find_system(h_vrsubsys,'regexp','on',...
        'referenceblock','dspindex/Multiport');
    h_multi = char(h_multi);
    
    % add a line for each multiport output (num_ac)
    for i=1:num_ac
        add_line(h_vrsubsys,[get_param(h_multi,'name') '/' num2str(i)],...
            [get_param(h_vrsink,'name') '/' num2str(2*i)],...
            'autorouting','on');
    end
    
    h_placehldr = find_system(h_vrsubsys,'referenceblock','vrlib/VR Placeholder');
    h_placehldr = char(h_placehldr);
    
    % add the VR Placeholder signal to all rotations on VR sink
    for i=1:num_ac
        add_line(h_vrsubsys,[get_param(h_placehldr,'name') '/1'],...
            [get_param(h_vrsink,'name') '/' num2str(2*i-1)],...
            'autorouting','on');
    end
end
