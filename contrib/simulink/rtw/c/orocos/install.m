function install()

setpath;

%disp('Orocos: configuring mex...')
%mex -setup

disp('Orocos: compiling blocks...')
cd taskcontext;
mex -v sfun_property.cpp;
mex -v sfun_vproperty.cpp;
mex -v sfun_inputport.cpp;
mex -v sfun_outputport.cpp;
mex -v sfun_vinputport.cpp;
mex -v sfun_voutputport.cpp;
mex -v sfun_iinputport.cpp;
mex -v sfun_ioutputport.cpp;
mex -v sfun_iproperty.cpp;
cd ..;

disp('Orocos: install completed.')
