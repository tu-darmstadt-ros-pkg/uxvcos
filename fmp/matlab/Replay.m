function hObject = Replay(directory)

if (nargin < 1)
    directory = pwd;
end

replay = struct();
replay.files = struct([]);
replay.directory = directory;

if exist(directory, 'dir')
    replay.files = dir([directory, filesep, '*.mat']);
end

if isempty(replay.files)
    error('Nix da zum Abspielen!');
end

for i = 1:size(replay.files)
    Meta = load([directory, filesep, replay.files(i).name]);
    replay.meta{i} = Meta;
    replay.versuch{i} = strrep(replay.files(i).name, '.mat', '');
    
    Start = replay.meta{i}.Start;
    Stop  = replay.meta{i}.Stop;
    
    if (ischar(Start))
        replay.start(i) = (datenum([replay.meta{i}.Datum ' ' Start], 'dd.mm.yyyy HH:HH:SS') - 719529) * 86400;
    else
        replay.start(i) = Start;
    end
    
    if (ischar(Stop))
        replay.stop(i) = (datenum([replay.meta{i}.Datum ' ' Stop], 'dd.mm.yyyy HH:HH:SS') - 719529) * 86400;
    else
        replay.stop(i) = Stop;
    end
    
    disp(['Versuch ' num2str(i) ':']);
    disp(['Start:  ' datestr(replay.start(i)/86400 + 719529, 'HH:MM:SS')]);
    disp(['Stop:   ' datestr(replay.stop(i)/86400 + 719529, 'HH:MM:SS')]);
    disp(replay.meta{i});
end

disp 'Press a key to continue'
pause

[replay.start, idx] = sort(replay.start);
replay.meta = replay.meta(idx);
replay.files = replay.files(idx);
replay.versuch = replay.versuch(idx);
replay.stop  = replay.stop(idx);

replay.i = 1;

hObject = FMP();
handles = guidata(hObject);
handles.Replay = replay;
guidata(hObject, handles);
