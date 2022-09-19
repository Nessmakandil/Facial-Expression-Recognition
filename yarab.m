function varargout = yarab(varargin)
% YARAB MATLAB code for yarab.fig
%      YARAB, by itself, creates a new YARAB or raises the existing
%      singleton*.
%
%      H = YARAB returns the handle to a new YARAB or the handle to
%      the existing singleton*.
%
%      YARAB('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in YARAB.M with the given input arguments.
%
%      YARAB('Property','Value',...) creates a new YARAB or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before yarab_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to yarab_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help yarab

% Last Modified by GUIDE v2.5 24-Apr-2019 02:04:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @yarab_OpeningFcn, ...
                   'gui_OutputFcn',  @yarab_OutputFcn, ...
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


% --- Executes just before yarab is made visible.
function yarab_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to yarab (see VARARGIN)

% Choose default command line output for yarab
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes yarab wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = yarab_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global a;
a = get(handles.edit1,'String');
assignin('base','a',a);
% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global datapath
 global testpath
datapath = 'D:\TrainingImage11';
 testpath = 'D:\TestImage11';
 global TestImage
 
 global a;
a = get(handles.edit1,'String');
assignin('base','a',a);

TestImage = strcat(testpath,'\',char(a),'.jpg');
axes (handles.axes1)
imshow(TestImage)
%%%%%%
D = dir(datapath);  % D is a Lx1 structure with 4 fields as: name,date,byte,isdir of all L files present in the directory 'datapath'
imgcount = 0;
for i=1 : size(D,1)
    if not(strcmp(D(i).name,'.')|strcmp(D(i).name,'..')|strcmp(D(i).name,'Thumbs.db'))
        imgcount = imgcount + 1; % Number of all images in the training database
    end
end
%%%%%%
X = [];
for i = 1 : imgcount
    str = strcat(datapath,'\',int2str(i),'.jpg');
    img = imread(str);
    img = rgb2gray(img);
    [r c] = size(img);
    temp = reshape(img',r*c,1);  %% Reshaping 2D images into 1D image vectors
                               %% here img' is used because reshape(A,M,N) function reads the matrix A columnwise
                               %% where as an image matrix is constructed with first N pixels as first row,next N in second row so on
    X = [X temp];                %% X,the image matrix with columnsgetting added for each image
end

m = mean(X,2); % Computing the average face image m = (1/P)*sum(Xj's)    (j = 1 : P)
imgcount = size(X,2);

A = [];
for i=1 : imgcount
    temp = double(X(:,i)) - m;
    A = [A temp];
end
L= A' * A;
[V,D]=eig(L);  %% V : eigenvector matrix  D : eigenvalue matrix

L_eig_vec = [];
for i = 1 : size(V,2) 
    if( D(i,i) > 1 )
        L_eig_vec = [L_eig_vec V(:,i)];
    end
end
eigenfaces = A * L_eig_vec;

projectimg = [ ];  % projected image vector matrix
for i = 1 : size(eigenfaces,2)
    temp = eigenfaces' * A(:,i);
    projectimg = [projectimg temp];
end
test_image = imread(TestImage);
test_image = test_image(:,:,1);
[r c] = size(test_image);
temp = reshape(test_image',r*c,1); % creating (MxN)x1 image vector from the 2D image
temp = double(temp)-m; % mean subtracted vector
projtestimg = eigenfaces'*temp; % projection of test image onto the facespace
euclide_dist = [ ];
for i=1 : size(eigenfaces,2)
    temp = (norm(projtestimg-projectimg(:,i)))^2;
    euclide_dist = [euclide_dist temp];
end
[euclide_dist_min recognized_index] = min(euclide_dist);
recognized_img = strcat(int2str(recognized_index),'.jpg');
selected_img = strcat(datapath,'\',recognized_img);
select_img = imread(selected_img);
axes (handles.axes2)
imshow(select_img)

global text;
if (recognized_img == strcat('1','.jpg'))
    text = 'Recognized Image is Nada Moharam - happy';
elseif (recognized_img ==  strcat('2','.jpg')) 
   text = 'Recognized Image is Nada Moharam - sad';
elseif (recognized_img ==  strcat('3','.jpg')) 
    text = 'Recognized Image is Nada Moharam - surprise';
elseif (recognized_img ==  strcat('4','.jpg')) 
    text = 'Recognized Image is Nada Moharam - disgust';
elseif (recognized_img ==  strcat('5','.jpg')) 
    text = 'Recognized Image is Nada Ahmed - happy';
elseif (recognized_img ==  strcat('6','.jpg')) 
   text = 'Recognized Image is Nada Ahmed - sad';
elseif (recognized_img ==  strcat('7','.jpg')) 
   text = 'Recognized Image is Nada Ahmed - surprise';
else     
    text = 'Recognized Image is Nada Ahmed - disgust' ; 
end

set(handles.text7,'string',text)

rimg = int2str(recognized_index)
setappdata(0,'rimg1',rimg);

global a testpath;
 testpath = 'D:\TestImage11';
a = get(handles.edit1,'String');
assignin('base','a',a);
img = strcat(testpath,'\',char(a),'.jpg');
im = imread (img);
faceDetector =vision.CascadeObjectDetector;
bbox=step(faceDetector,im);
subimage=imcrop(im,bbox);
axes (handles.axes4);
imshow(subimage);
imwrite(subimage,strcat(im,'.jpg'));


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global a testpath;
 testpath = 'D:\TestImage11';
a = get(handles.edit1,'String');
assignin('base','a',a);
im = strcat(testpath,'\',char(a),'.jpg');
img = imread (im);
editedphoto = rgb2gray(img);
axes (handles.axes3)
imshow(editedphoto)



% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global a testpath;
 testpath = 'D:\TestImage11';
a = get(handles.edit1,'String');
assignin('base','a',a);
im = strcat(testpath,'\',char(a),'.jpg');
img = imread (im);
img = rgb2gray (img);
editedphoto = histeq(img);
axes (handles.axes3)
imhist(editedphoto,64);

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global a testpath;
 testpath = 'D:\TestImage11';
a = get(handles.edit1,'String');
assignin('base','a',a);
im = strcat(testpath,'\',char(a),'.jpg');
img = imread (im);
editedphoto = imnoise(img,'Salt & Pepper',.1);
axes (handles.axes3)
imshow(editedphoto)


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
m = getappdata(0,'rimg1');
datapath = 'D:\TrainingImage11';
im = strcat(datapath,'\',char(m),'.jpg');
img = imread (im);
editedphoto = rgb2gray(img);
axes (handles.axes3)
imshow(editedphoto)



% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
m = getappdata(0,'rimg1');
datapath = 'D:\TrainingImage11';
im = strcat(datapath,'\',char(m),'.jpg');
img = imread (im);
img = rgb2gray (img);
editedphoto = histeq(img);
axes (handles.axes3)
imhist(editedphoto,64);



% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
m = getappdata(0,'rimg1');
datapath = 'D:\TrainingImage11';
im = strcat(datapath,'\',char(m),'.jpg');
img = imread (im);
editedphoto = imnoise(img,'Salt & Pepper',.1);
axes (handles.axes3)
imshow(editedphoto)


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
m = getappdata(0,'rimg1');
datapath = 'D:\TrainingImage11';
im = strcat(datapath,'\',char(m),'.jpg');
img = imread (im);
editedphoto = im2bw(img);
axes (handles.axes3)
imshow(editedphoto)



% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global a testpath;
 testpath = 'D:\TestImage11';
a = get(handles.edit1,'String');
assignin('base','a',a);
im = strcat(testpath,'\',char(a),'.jpg');
img = imread (im);
editedphoto = im2bw(img);
axes (handles.axes3)
imshow(editedphoto)


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global a testpath;
 testpath = 'D:\TestImage11';
a = get(handles.edit1,'String');
assignin('base','a',a);
img = strcat(testpath,'\',char(a),'.jpg');
im = imread (img);
faceDetector =vision.CascadeObjectDetector;
bbox=step(faceDetector,im);
subimage=imcrop(im,bbox);
axes (handles.axes4);
imshow(subimage);
imwrite(subimage,strcat(im,'.jpg'));
