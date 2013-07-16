function varargout = Konfigurieren(varargin)
% KONFIGURIEREN M-file for Konfigurieren.fig
%      KONFIGURIEREN, by itself, creates a new KONFIGURIEREN or raises the existing
%      singleton*.
%
%      H = KONFIGURIEREN returns the handle to a new KONFIGURIEREN or the handle to
%      the existing singleton*.
%
%      KONFIGURIEREN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in KONFIGURIEREN.M with the given input arguments.
%
%      KONFIGURIEREN('Property','Value',...) creates a new KONFIGURIEREN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Konfigurieren_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Konfigurieren_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Konfigurieren

% Last Modified by GUIDE v2.5 25-Jul-2010 17:26:46

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Konfigurieren_OpeningFcn, ...
                   'gui_OutputFcn',  @Konfigurieren_OutputFcn, ...
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


% --- Executes just before Konfigurieren is made visible.
function Konfigurieren_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Konfigurieren (see VARARGIN)

% Choose default command line output for Konfigurieren
handles.output = hObject;
handles.parent = 0;

if size(varargin) > 0
    handles.parent = varargin{1};
    try
        set(handles.Name, 'String', get(handles.parent.Name, 'String'));
        set(handles.Abfluggewicht, 'String', handles.parent.Daten.Abfluggewicht);
    catch
    end
end

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Konfigurieren wait for user response (see UIRESUME)
uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Konfigurieren_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

delete(hObject);



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


% --- Executes on button press in ButtonAbbrechen.
function ButtonAbbrechen_Callback(hObject, eventdata, handles)
% hObject    handle to ButtonAbbrechen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.output = struct([]);
guidata(hObject, handles);
uiresume(handles.figure1);

% --- Executes on button press in ButtonOK.
function ButtonOK_Callback(hObject, eventdata, handles)
% hObject    handle to ButtonOK (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.output = struct();
handles.output.Name = get(handles.Name, 'String');
handles.output.Abfluggewicht = get(handles.Abfluggewicht, 'String');
guidata(hObject, handles);

uiresume(handles.figure1);



function Abfluggewicht_Callback(hObject, eventdata, handles)
% hObject    handle to Abfluggewicht (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Abfluggewicht as text
%        str2double(get(hObject,'String')) returns contents of Abfluggewicht as a double


% --- Executes during object creation, after setting all properties.
function Abfluggewicht_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Abfluggewicht (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
