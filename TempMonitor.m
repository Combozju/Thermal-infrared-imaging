function varargout = TempMonitor(varargin)
% TEMPMONITOR MATLAB code for TempMonitor.fig
%      TEMPMONITOR, by itself, creates a new TEMPMONITOR or raises the existing
%      singleton*.
%
%      H = TEMPMONITOR returns the handle to a new TEMPMONITOR or the handle to
%      the existing singleton*.
%
%      TEMPMONITOR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TEMPMONITOR.M with the given input arguments.
%
%      TEMPMONITOR('Property','Value',...) creates a new TEMPMONITOR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before TempMonitor_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to TempMonitor_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help TempMonitor

% Last Modified by GUIDE v2.5 18-Jun-2022 12:37:18

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @TempMonitor_OpeningFcn, ...
                   'gui_OutputFcn',  @TempMonitor_OutputFcn, ...
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


% --- Executes just before TempMonitor is made visible.
function TempMonitor_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to TempMonitor (see VARARGIN)

% Choose default command line output for TempMonitor
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes TempMonitor wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = TempMonitor_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in ReadFig.
function ReadFig_Callback(hObject, eventdata, handles)
% ��ͼ
% hObject    handle to ReadFig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% global FileName 
global I
[FileName,PathName] = uigetfile('.jpg');    % �򿪴���
TotalPath = [PathName, '\', FileName];  % ����·��
IRGB = imread(TotalPath);   % ��ͼ
% I = rgb2gray(IRGB);
r = IRGB(:,:,1);
g = IRGB(:,:,2);
b = IRGB(:,:,3);
I = .299*r + .587*g + .114*b;   % ת�Ҷ�ͼ
set(handles.text5,'String',strcat(TotalPath,' loaded.'))    % ��ʾ����

axis(handles.FigOri);
imshow(I)
title('��ɫͼ��ҶȻ�')    % ���Ҷ�ͼ



% --- Executes on button press in AddNoise.
function AddNoise_Callback(hObject, eventdata, handles)
% ��������ť
% hObject    handle to AddNoise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global I NoiseType J
if isempty(NoiseType)   % �ж��Ƿ�ѡ����������
    msgbox('δѡ����������')
    error('δѡ����������')
end
t = get(handles.NoisePara, 'String');   
if iscell(t)
    tt = t{1};    
elseif isa(t, 'string') || isa(t, 'char')
    tt = t;
end
NoisePara = str2num(tt);    % ������������
% NoisePara = str2num(get(handles.NoisePara, 'String'));
if strcmp(NoiseType, 'Gauss')   % ��˹����
    type = 'gaussian';
elseif strcmp(NoiseType, 'Salt')   % ��������
    type = 'salt & pepper';
else % δѡ�񱨴�
    msgbox('δѡ����������')
    error('δѡ����������')
end
J = imnoise(I,type,NoisePara);  % ����

axis(handles.FigOri);    % ��ͼ
plot([1 2], [3 4])
imshow(J)
title([NoiseType,'������������', num2str(NoisePara)])


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1
global NoiseType    % ���������˵�ȷ����������
NoiseSel = get(handles.popupmenu1, 'Value');
switch NoiseSel
    case 1
        NoiseType = 'None';
    case 2
        NoiseType = 'Gauss';
    case 3
        NoiseType = 'Salt';
end


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





function FigPath_Callback(hObject, eventdata, handles)
% hObject    handle to FigPath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FigPath as text
%        str2double(get(hObject,'String')) returns contents of FigPath as a double


% --- Executes during object creation, after setting all properties.
function FigPath_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FigPath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function NoisePara_Callback(hObject, eventdata, handles)
% hObject    handle to NoisePara (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of NoisePara as text
%        str2double(get(hObject,'String')) returns contents of NoisePara as a double


% --- Executes during object creation, after setting all properties.
function NoisePara_CreateFcn(hObject, eventdata, handles)
% hObject    handle to NoisePara (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Filter.
function Filter_Callback(hObject, eventdata, handles)
% �˲���ť
% hObject    handle to Filter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global FilterType J FiltJ
if isempty(FilterType)  % δѡ���˲�������
    msgbox('δѡ���˲�������')
    error('δѡ���˲�������')
end
t = get(handles.FilterPara, 'String');
if iscell(t)
    tt = t{1};    
elseif isa(t, 'string') || isa(t, 'char')
    tt = t;
else
    msgbox('δѡ��ƽ������')
    error('δѡ��ƽ������')
end
FilterPara = str2num(tt);   % ���˲�������
if strcmp(FilterType, 'Average')   % ��ֵ�˲�
    H = fspecial('average',FilterPara);
    FiltJ = imfilter(J, H);
elseif strcmp(FilterType, 'Median')   % ��ֵ�˲�
    FiltJ = medfilt3(J);
end
axis(handles.FigOri);   % ��ͼ
imshow(FiltJ)
title([FilterType,'���룬������', num2str(FilterPara)])


% --- Executes on selection change in ChooseFilter.
function ChooseFilter_Callback(hObject, eventdata, handles)
% hObject    handle to ChooseFilter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ChooseFilter contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ChooseFilter
global FilterType % ���������˵�ѡ���˲���
FilterSel = get(handles.ChooseFilter, 'Value');
switch FilterSel
    case 1
        FilterType = 'None';
    case 2
        FilterType = 'Average';
    case 3
        FilterType = 'Median';
end


% --- Executes during object creation, after setting all properties.
function ChooseFilter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ChooseFilter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function FilterPara_Callback(hObject, eventdata, handles)
% hObject    handle to FilterPara (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FilterPara as text
%        str2double(get(hObject,'String')) returns contents of FilterPara as a double


% --- Executes during object creation, after setting all properties.
function FilterPara_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FilterPara (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Check.
function Check_Callback(hObject, eventdata, handles)
% hObject    handle to Check (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global FiltJ

t = get(handles.Temp, 'String');  % �����¶���ֵ�ͱ���
if iscell(t)
    tt = t{1};    
elseif isa(t, 'string') || isa(t, 'char')
    tt = t;
else
    msgbox('δ�����¶���ֵ')
    error('δ�����¶���ֵ')
end
SetTemp = str2num(tt);% ��ȡ�¶���ֵ

SubFig = uint32(FiltJ(:, 1:642));   % ��ȡ������ɫ������ͼƬ����
% t1 = .299*249 + .587*250 + .114*252;
% t2 = .299*254 + .587*253 + .114*223;
% k = (28.9 - 28.73)/(t1 - t2);
% b = 28.9 - k*t1;    % �õ��Ҷ����¶ȵ����Թ�ϵ

% Temp = k*SubFig + b;
[m,n] = size(SubFig);
[mm, nn] = size(FiltJ);
TempFig = zeros(mm,nn);   % ����¶ȵ�ͼ�����������ֵ����ʾ��ɫ��������ʾ��ɫ

% LocalFigList = zeros(m,n);
for i = 2:m-1
    for j = 2:n-1
        % �Ź����������ҶȾ�ֵ
        LocalFig = (SubFig(i-1, j-1) + SubFig(i, j-1) + SubFig(i+1, j-1) + ...
            SubFig(i-1, j) + SubFig(i, j) + SubFig(i+1, j) + ...
            SubFig(i-1, j + 1) + SubFig(i, j + 1) + SubFig(i+1, j + 1))/9;
%         LocalTemp = k*LocalFig + b; % ��Ӧ���¶�
        LocalTemp = (44-14) * LocalFig/255 + 14; % ��Ӧ���¶�
%         LocalFigList(i,j) = LocalFig;
        if LocalTemp > SetTemp % ������ֵ���趨Ϊ��ɫ
            TempFig(i-1:i+1,j-1:j+1) = 255;
        end
    end
end

axis(handles.FigOri);   % ��ͼ
imshow(uint8(TempFig))
title(['�쳣��⣬�趨�¶ȣ�', num2str(SetTemp)])









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


% --- Executes during object creation, after setting all properties.
function FigOri_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FigOri (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate FigOri
