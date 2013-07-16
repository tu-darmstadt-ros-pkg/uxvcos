function setpath()

disp('Orocos: setting path.')

addpath( pwd );
addpath([pwd, '/taskcontext']);
savepath;

disp('Orocos: done.')
