function varargout = FMP(varargin)
% FMP M-file for FMP.fig
%      FMP, by itself, creates a new FMP or raises the existing
%      singleton*.
%
%      H = FMP returns the handle to a new FMP or the handle to
%      the existing singleton*.
%
%      FMP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FMP.M with the given input arguments.
%
%      FMP('Property','Value',...) creates a new FMP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before FMP_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to FMP_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help FMP

% Last Modified by GUIDE v2.5 16-May-2012 09:24:07

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @FMP_OpeningFcn, ...
                   'gui_OutputFcn',  @FMP_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before FMP is made visible.
function FMP_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to FMP (see VARARGIN)

% Choose default command line output for FMP
handles.output = hObject;

if (isfield(handles, 'initialized') && handles.initialized); return; end

handles.args = varargin;
handles.ros = [];

handles.Horizont  = Horizont(handles.Horizont);
handles.Kompass   = Kompass(handles.Kompass);
handles.Karte     = Karte(handles.Karte);
handles.AlphaBeta = AlphaBeta(handles.AlphaBeta);
handles.Ruderwege = Ruderwege(handles.AileronElevator, handles.Rudder, handles.ElevatorTrim);
handles.Spannung  = Spannung(handles.Spannung);
handles.StartLandeZeit = StartLandeZeit(handles.Startzeit, handles.Flugzeit, handles.Landezeit);

handles.Daten     = struct();
handles.Versuche  = Versuche;
set(handles.Versuch, 'String', {handles.Versuche{:,1}});
set(handles.Bemerkung, 'String', []);

handles.logging   = 0;
handles.initialized = 1;

handles.UpdateTimer = timer('Period', 0.001, 'ExecutionMode', 'fixedSpacing', 'TimerFcn', @(source, event) FMP_UpdateFcn(hObject, source, event));

% Load message types
fmp_msgs
nav_msgs
sensor_msgs
geometry_msgs
hector_uav_msgs
rosgraph_msgs

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes FMP wait for user response (see UIRESUME)
% uiwait(handles.FMP);


% --- Outputs from this function are returned to the command line.
function varargout = FMP_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in ButtonVerbindenTrennen.
function ButtonVerbindenTrennen_Callback(hObject, eventdata, handles)
% hObject    handle to ButtonVerbindenTrennen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


if (isempty(handles.ros))
    try
        handles.ros = struct();
        handles.ros.state = ros.Subscriber('state', 'nav_msgs/Odometry', 100);
        %handles.ros.state.addlistener('Callback', @(source, event) Update(handles, source, event));
        handles.ros.fmp   = ros.Subscriber('fmp', 'fmp_msgs/SensorData', 100);
        %handles.ros.fmp.addlistener('Callback', @(source, event) Update(handles, source, event));
        handles.ros.global   = ros.Subscriber('global', 'sensor_msgs/NavSatFix', 1);
        %handles.ros.global.addlistener('Callback', @(source, event) Update(handles, source, event));
        handles.ros.fix_velocity = ros.Subscriber('fix_velocity', 'geometry_msgs/Vector3Stamped', 1);
        %handles.ros.fix_velocity.addlistener('Callback', @(source, event) Update(handles, source, event));
        handles.ros.magnetic = ros.Subscriber('magnetic', 'geometry_msgs/Vector3Stamped', 1);
        handles.ros.supply = ros.Subscriber('supply', 'hector_uav_msgs/Supply', 1);
        handles.ros.status = ros.Subscriber('navigation_status', 'rosgraph_msgs/Log', 1);
    catch error
        objects = fieldnames(handles.ros);
        for i = 1:length(objects)
            delete(handles.ros.(objects{i}));
        end
        handles.ros = [];
        
        errordlg(error.message, 'Fehler');
        return;
    end
    
    % Update handles structure
    guidata(hObject, handles);    

    Update(hObject);
    start(handles.UpdateTimer);
    
    set(handles.ButtonVerbindenTrennen, ...
        'String', 'Trennen', ...
        'BackgroundColor', 'g' ...
    );

    set(handles.ButtonStartStop, 'Enable', 'on');
else
    %button = questdlg('Wirklich trennen?', 'Trennen', 'Ja', 'Nein', 'Nein');
    %if (~strcmp(button, 'Ja')); return; end
    
    if (handles.logging ~= 0); ButtonStartStop_Callback(hObject, eventdata, handles); end
    handles.logging = 0;
    
    stop(handles.UpdateTimer);
    
    objects = fieldnames(handles.ros);
    for i = 1:length(objects)
        delete(handles.ros.(objects{i}));
    end
    handles.ros = [];

    set(handles.ButtonVerbindenTrennen, ...
        'String', 'Verbinden', ...
        'BackgroundColor', 'r' ...
    );

    set(handles.ButtonStartStop, 'Enable', 'off');
end

% Update handles structure
guidata(hObject, handles);

% --- Update the GUI elements.
function FMP_UpdateFcn(hObject, source, event)
try
    Update(hObject, source, event)
catch error
    disp(error);
    disp(error.stack(1));
end
    

% --- Executes on button press in ButtonStartStop.
function ButtonStartStop_Callback(hObject, eventdata, handles)
% hObject    handle to ButtonStartStop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if (handles.logging == 0)
    handles.Daten.Name = get(handles.Name, 'String');
    handles.Daten.Versuch = handles.Versuche{get(handles.Versuch, 'Value'), 2};
    handles.Daten.Bemerkung = get(handles.Bemerkung, 'String');
    
    try
        handles.logging = Logging(handles.Daten);
        handles.logging.start();
    catch error
        errordlg(error.message, 'Fehler');
        return;
    end

    set(handles.ButtonStartStop, ...
        'String', 'Stop', ...
        'BackgroundColor', 'g' ...
    );
    set([handles.Versuch handles.Bemerkung handles.ButtonKonfigurieren], 'Enable', 'off');
else
    delete(handles.logging);
    handles.logging = 0;
    
    set(handles.ButtonStartStop, ...
        'String', 'Start', ...
        'BackgroundColor', 'r' ...
    );
    set([handles.Versuch handles.Bemerkung handles.ButtonKonfigurieren], 'Enable', 'on');
end

% Update handles structure
guidata(hObject, handles);

function Name_Callback(hObject, eventdata, handles)
% hObject    handle to Name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Name as text
%        str2double(get(hObject,'String')) returns contents of Name as a double


% --- Executes during object creation, after setting all properties.
function Name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function NavStatus_Callback(hObject, eventdata, handles)
% hObject    handle to NavStatus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of NavStatus as text
%        str2double(get(hObject,'String')) returns contents of NavStatus as a double


% --- Executes during object creation, after setting all properties.
function NavStatus_CreateFcn(hObject, eventdata, handles)
% hObject    handle to NavStatus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function Temp_Callback(hObject, eventdata, handles)
% hObject    handle to Temp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Temp as text
%        str2double(get(hObject,'String')) returns contents of Temp as a double


% --- Executes during object creation, after setting all properties.
function Temp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Temp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Humi_Callback(hObject, eventdata, handles)
% hObject    handle to Humi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Humi as text
%        str2double(get(hObject,'String')) returns contents of Humi as a double


% --- Executes during object creation, after setting all properties.
function Humi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Humi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function P_stat_Callback(hObject, eventdata, handles)
% hObject    handle to P_stat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of P_stat as text
%        str2double(get(hObject,'String')) returns contents of P_stat as a double


% --- Executes during object creation, after setting all properties.
function P_stat_CreateFcn(hObject, eventdata, handles)
% hObject    handle to P_stat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function P_dyn_Callback(hObject, eventdata, handles)
% hObject    handle to P_dyn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of P_dyn as text
%        str2double(get(hObject,'String')) returns contents of P_dyn as a double


% --- Executes during object creation, after setting all properties.
function P_dyn_CreateFcn(hObject, eventdata, handles)
% hObject    handle to P_dyn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function IAS_Callback(hObject, eventdata, handles)
% hObject    handle to IAS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of IAS as text
%        str2double(get(hObject,'String')) returns contents of IAS as a double


% --- Executes during object creation, after setting all properties.
function IAS_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IAS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function TAS_Callback(hObject, eventdata, handles)
% hObject    handle to TAS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of TAS as text
%        str2double(get(hObject,'String')) returns contents of TAS as a double


% --- Executes during object creation, after setting all properties.
function TAS_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TAS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function SystemTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SystemTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on selection change in Versuch.
function Versuch_Callback(hObject, eventdata, handles)
% hObject    handle to Versuch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Versuch contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Versuch


% --- Executes during object creation, after setting all properties.
function Versuch_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Versuch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit16_Callback(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit16 as text
%        str2double(get(hObject,'String')) returns contents of edit16 as a double


% --- Executes during object creation, after setting all properties.
function edit16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in ButtonKonfigurieren.
function ButtonKonfigurieren_Callback(hObject, eventdata, handles)
% hObject    handle to ButtonKonfigurieren (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Daten = Konfigurieren(handles);
if (~isempty(handles.Daten))
    set(handles.Name, 'String', handles.Daten.Name);
    guidata(hObject, handles);
end


function Latitude_Callback(hObject, eventdata, handles)
% hObject    handle to Latitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Latitude as text
%        str2double(get(hObject,'String')) returns contents of Latitude as a double


% --- Executes during object creation, after setting all properties.
function Latitude_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Latitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function GPSStatus_Callback(hObject, eventdata, handles)
% hObject    handle to GPSStatus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of GPSStatus as text
%        str2double(get(hObject,'String')) returns contents of GPSStatus as a double


% --- Executes during object creation, after setting all properties.
function GPSStatus_CreateFcn(hObject, eventdata, handles)
% hObject    handle to GPSStatus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Longitude_Callback(hObject, eventdata, handles)
% hObject    handle to Longitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Longitude as text
%        str2double(get(hObject,'String')) returns contents of Longitude as a double


% --- Executes during object creation, after setting all properties.
function Longitude_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Longitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function GPSAltitude_Callback(hObject, eventdata, handles)
% hObject    handle to GPSAltitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of GPSAltitude as text
%        str2double(get(hObject,'String')) returns contents of GPSAltitude as a double


% --- Executes during object creation, after setting all properties.
function GPSAltitude_CreateFcn(hObject, eventdata, handles)
% hObject    handle to GPSAltitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Bemerkung_Callback(hObject, eventdata, handles)
% hObject    handle to Bemerkung (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Bemerkung as text
%        str2double(get(hObject,'String')) returns contents of Bemerkung as a double


% --- Executes during object creation, after setting all properties.
function Bemerkung_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Bemerkung (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function GPSSat_Callback(hObject, eventdata, handles)
% hObject    handle to GPSSat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of GPSSat as text
%        str2double(get(hObject,'String')) returns contents of GPSSat as a double


% --- Executes during object creation, after setting all properties.
function GPSSat_CreateFcn(hObject, eventdata, handles)
% hObject    handle to GPSSat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function GPSCourse_Callback(hObject, eventdata, handles)
% hObject    handle to GPSCourse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of GPSCourse as text
%        str2double(get(hObject,'String')) returns contents of GPSCourse as a double


% --- Executes during object creation, after setting all properties.
function GPSCourse_CreateFcn(hObject, eventdata, handles)
% hObject    handle to GPSCourse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function GPSSpeed_Callback(hObject, eventdata, handles)
% hObject    handle to GPSSpeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of GPSSpeed as text
%        str2double(get(hObject,'String')) returns contents of GPSSpeed as a double


% --- Executes during object creation, after setting all properties.
function GPSSpeed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to GPSSpeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Geschwindigkeit_Callback(hObject, eventdata, handles)
% hObject    handle to Geschwindigkeit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Geschwindigkeit as text
%        str2double(get(hObject,'String')) returns contents of Geschwindigkeit as a double


% --- Executes during object creation, after setting all properties.
function Geschwindigkeit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Geschwindigkeit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Hoehe_Callback(hObject, eventdata, handles)
% hObject    handle to Hoehe (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Hoehe as text
%        str2double(get(hObject,'String')) returns contents of Hoehe as a double


% --- Executes during object creation, after setting all properties.
function Hoehe_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Hoehe (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Heading_Callback(hObject, eventdata, handles)
% hObject    handle to Heading (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Heading as text
%        str2double(get(hObject,'String')) returns contents of Heading as a double


% --- Executes during object creation, after setting all properties.
function Heading_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Heading (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Rate_Callback(hObject, eventdata, handles)
% hObject    handle to Rate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Rate as text
%        str2double(get(hObject,'String')) returns contents of Rate as a double


% --- Executes during object creation, after setting all properties.
function Rate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Rate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when user attempts to close FMP.
function FMP_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to FMP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

try
    if (handles.logging ~= 0); ButtonStartStop_Callback(hObject, eventdata, handles); end
    if (handles.ros ~= 0); ButtonVerbindenTrennen_Callback(hObject, eventdata, handles); end
end

delete(handles.UpdateTimer)

% Hint: delete(hObject) closes the figure
delete(hObject);


% --- Executes when FMP is resized.
function FMP_ResizeFcn(hObject, eventdata, handles)
% hObject    handle to FMP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over Geschwindigkeit.
function Geschwindigkeit_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to Geschwindigkeit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(hObject, 'UserData', get(hObject, 'UserData') + 1);


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over Hoehe.
function Hoehe_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to Hoehe (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(hObject, 'UserData', get(hObject, 'UserData') + 1);


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over Rate.
function Rate_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to Rate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(hObject, 'UserData', get(hObject, 'UserData') + 1);


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over GPSSpeed.
function GPSSpeed_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to GPSSpeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(hObject, 'UserData', get(hObject, 'UserData') + 1);


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over IAS.
function IAS_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to IAS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(hObject, 'UserData', get(hObject, 'UserData') + 1);


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over TAS.
function TAS_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to TAS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(hObject, 'UserData', get(hObject, 'UserData') + 1);


% --- Executes during object creation, after setting all properties.
function Track_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Track (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over GPSAltitude.
function GPSAltitude_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to GPSAltitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(hObject, 'UserData', get(hObject, 'UserData') + 1);
