function Update(hObject, source, event)

if (nargin == 0); hObject = gcbo; end

if (ishandle(hObject))
    handles = guidata(hObject);
else
    handles = hObject;
end

global Data;
global LastUpdateTime;

updated.state = 0;
updated.imu = 0;
updated.fmp = 0;
updated.global = 0;
updated.fix_velocity = 0;
updated.magnetic = 0;
updated.supply = 0;
updated.status = 0;

if (isempty(Data))
    Standardwerte;
end

if (nargin <= 1)
    % Update all values
    updated.state = 1;
    updated.imu = 1;
    updated.fmp = 1;
    updated.global = 1;
    updated.fix_velocity = 1;
    updated.magnetic = 1;
    updated.supply = 1;
    updated.status = 1;
    LastUpdateTime = [];
    Data.timestamp = 0;
    handles.Karte.reset();
end

%% Read new data
updated.any = 1;
while updated.any
    updated.any = 0;

    data = handles.ros.state.poll(0);
    if (~isempty(data))
        if (Data.timestamp ~= 0)
            dt = data.header.stamp - Data.timestamp;
            if (dt > 0.015)
                warning('FMP:missed_state', ['Missed a state message (dt = ' num2str(dt) ')']);
            end
        else
            dt = 0.0;
        end
        Data.timestamp = data.header.stamp;
        Data.state = data;
        updated.state = 1;
        updated.any = 1;
        %data = handles.ros.state.poll(0);

        % Start-/Landezeit-Ermittlung
        geschwindigkeit = sqrt(Data.state.twist.twist.linear.x^2 + Data.state.twist.twist.linear.y^2);
        if (isnan(Data.v_mittel)); Data.v_mittel = geschwindigkeit; end
        T = 10.0;
        Data.v_mittel = (T * Data.v_mittel + 0.01 * geschwindigkeit) / (T + dt);
        if (Data.start == 0 && Data.v_mittel > 10.0)
            Data.start = Data.timestamp;
            set(handles.Startzeit, 'String', datestr(Data.start/86400, 'HH:MM'));
        end
        if (Data.start > 0 && Data.landung == 0)
            Data.flugzeit = Data.timestamp - Data.start;
            set(handles.Flugzeit, 'String', datestr(Data.flugzeit/86400, 'HH:MM'));
        end
        if (Data.start > 0 && Data.v_mittel < 8.0)
            Data.landung = Data.timestamp;
            set(handles.Landezeit, 'String', datestr(Data.landung/86400, 'HH:MM'));
        end
        
        % Replay
        if (isfield(handles, 'Replay'))
            if ~isfield(handles.Replay, 'i')
                handles.Replay.i = 1;
            end
            
            if (handles.Replay.i <= length(handles.Replay.files))
                if ((handles.logging == 0) && (Data.timestamp >= handles.Replay.start(handles.Replay.i)))
                    if (Data.timestamp < handles.Replay.stop(handles.Replay.i))
                        disp(['[' datestr(Data.timestamp/86400, 'HH:MM:SS UTC') '] Starte Replay von Versuch ' handles.Replay.versuch{handles.Replay.i}]);
                        handles.logging = Logging(handles.Replay.meta{handles.Replay.i}, handles.Replay.directory);
                        handles.logging.start();
                    else
                        handles.Replay.i = handles.Replay.i + 1;
                    end
                    guidata(hObject, handles);
                elseif ((handles.logging ~= 0) && (Data.timestamp >= handles.Replay.stop(handles.Replay.i)))
                    disp(['[' datestr(Data.timestamp/86400, 'HH:MM:SS UTC') '] Stoppe Replay von Versuch ' handles.Replay.versuch{handles.Replay.i}]);
                    delete(handles.logging);
                    handles.logging = 0;
                    handles.Replay.i = handles.Replay.i + 1;
                    guidata(hObject, handles);
                end
            end
                    
        end            

        % Logging
        if (isa(handles.logging, 'Logging'))
            handles.logging.log(source, event, updated);
        end
    end

    data = handles.ros.fmp.poll(0);
    if (~isempty(data))
        if (Data.fmp.header.stamp ~= 0)
            dt = data.header.stamp - Data.fmp.header.stamp;
            if (dt > 0.03)
                warning('FMP:missed_fmp', ['Missed a fmp message (dt = ' num2str(dt) ')']);
            end
        end

        Data.fmp = data;
        updated.fmp = 1;
        updated.any = 1;
        %data = handles.ros.fmp.poll(0);
    end

    data = handles.ros.global.poll(0);
    if (~isempty(data))
        Data.global = data;
        updated.global = 1;
        updated.any = 1;
        %data = handles.ros.global.poll(0);
    end

    data = handles.ros.fix_velocity.poll(0);
    if (~isempty(data))
        Data.fix_velocity = data;
        updated.fix_velocity = 1;
        updated.any = 1;
        %data = handles.ros.fix_velocity.poll(0);
    end

    data = handles.ros.magnetic.poll(0);
    if (~isempty(data))
        Data.magnetic = data;
        updated.magnetic = 1;
        updated.any = 1;
        %data = handles.ros.magnetic.poll(0);
    end

    data = handles.ros.supply.poll(0);
    if (~isempty(data))
        Data.supply = data;
        updated.supply = 1;
        updated.any = 1;
        %data = handles.ros.supply.poll(0);
    end

    data = handles.ros.status.poll(0);
    if (~isempty(data))
        Data.status = data;
        updated.status = 1;
        updated.any = 1;
        %data = handles.ros.status.poll(0);
    end
end

%% Update GUI elements

if (~isempty(LastUpdateTime) && Data.timestamp >= LastUpdateTime && Data.timestamp < LastUpdateTime + .2); return; end
LastUpdateTime = Data.timestamp;

%set(handles.SystemTime, 'String', datestr(Data.timestamp/86400, 'HH:MM:SS.FFF'));
set(handles.UTC, 'String', datestr(Data.timestamp/86400 + 719529, 'dd.mm.yyyy HH:MM:SS.FFF UTC'));

%drawnow; return;

if (updated.state)
    [Data.euler(1) Data.euler(2) Data.euler(3)] = quat2angle([Data.state.pose.pose.orientation.w Data.state.pose.pose.orientation.x Data.state.pose.pose.orientation.y Data.state.pose.pose.orientation.z]);
    Data.euler = Data.euler .* [-1 -1 1];
    handles.Horizont.update(Data.euler(3), Data.euler(2));

    geschwindigkeit = sqrt(Data.state.twist.twist.linear.x^2 + Data.state.twist.twist.linear.y^2);
    switch(get(handles.Geschwindigkeit, 'UserData'))
        case 0; set(handles.Geschwindigkeit, 'String', sprintf('%.1f km/h', geschwindigkeit * 3.6));
        case 1; set(handles.Geschwindigkeit, 'String', sprintf('%.1f kt', geschwindigkeit * 3.6 / 1.852));
        otherwise; set(handles.Geschwindigkeit, 'UserData', 0);
    end
    set(handles.Heading,      'String', sprintf('%.0f °', mod(Data.euler(1) * 180/pi, 360)));

    Data.course = -atan2(Data.state.twist.twist.linear.y, Data.state.twist.twist.linear.x);
    if (geschwindigkeit < 1); Data.course = nan; end
    set(handles.Track,        'String', sprintf('%.0f °', mod(Data.course * 180/pi, 360)));
    handles.Kompass.update(Data.compassheading, Data.euler(1), Data.course);

    switch(get(handles.Rate, 'UserData'))
        case 0; set(handles.Rate,            'String', sprintf('%.1f m/s', Data.state.twist.twist.linear.z));
        case 1; set(handles.Rate,            'String', sprintf('%.0f ft/min', Data.state.twist.twist.linear.z * 60 / .3048));
        otherwise; set(handles.Rate, 'UserData', 0);
    end
end

if (updated.global)
    handles.Karte.update(Data.timestamp, Data.global.latitude, Data.global.longitude);

    switch(Data.global.status.status)
        case -1
            set(handles.GPSStatus, 'String', 'No Fix', 'BackgroundColor', 'red');
        case 0
            set(handles.GPSStatus, 'String', 'Fix', 'BackgroundColor', 'green');
        case 1
            set(handles.GPSStatus, 'String', 'Fix (SBAS)', 'BackgroundColor', 'green');
        case 2
            set(handles.GPSStatus, 'String', 'Fix (GBAS)', 'BackgroundColor', 'green');
    end
    %set(handles.GPSSat, 'String', '?');

    switch(get(handles.Hoehe, 'UserData'))
        case 0; set(handles.Hoehe,           'String', sprintf('%.1f m', Data.global.altitude));
        case 1; set(handles.Hoehe,           'String', sprintf('%.0f ft', Data.global.altitude / 0.3048));
        otherwise; set(handles.Hoehe, 'UserData', 0);
    end

    Latitude  = Data.global.latitude; % * 180/pi;
    Longitude = Data.global.longitude; % * 180/pi;
    NS = 'N'; if (Latitude < 0); NS = 'S'; end
    EW = 'O'; if (Longitude < 0); EW = 'W'; end
    set(handles.Latitude, 'String', sprintf('%02d %02d.%03d %s', fix(Latitude), fix(mod(Latitude * 60, 60)), fix(mod(Latitude * 60000, 1000)), NS));
    set(handles.Longitude, 'String', sprintf('%03d %02d.%03d %s', fix(Longitude), fix(mod(Longitude * 60, 60)), fix(mod(Longitude * 60000, 1000)), EW));
    switch(get(handles.GPSAltitude, 'UserData'))
        case 0; set(handles.GPSAltitude, 'String', sprintf('%.0f m', Data.global.altitude));
        case 1; set(handles.GPSAltitude, 'String', sprintf('%.0f ft', Data.global.altitude/0.3048));
        otherwise; set(handles.GPSAltitude, 'UserData', 0);
    end
end

if (updated.fix_velocity)
    course = -atan2(Data.fix_velocity.vector.y, Data.fix_velocity.vector.x);
    speed  = sqrt(Data.fix_velocity.vector.x^2 + Data.fix_velocity.vector.y^2);
    set(handles.GPSCourse, 'String', sprintf('%.0f °', mod(course * 180/pi, 360)));
    switch(get(handles.GPSSpeed, 'UserData'))
        case 0; set(handles.GPSSpeed, 'String', sprintf('%.1f km/h', speed * 3.6));
        case 1; set(handles.GPSSpeed, 'String', sprintf('%.1f kt', speed * 3.6 / 1.852));
        otherwise; set(handles.GPSSpeed, 'UserData', 0);
    end
end

if (updated.magnetic)
    Data.compassheading = atan2(Data.magnetic.vector.y, Data.magnetic.vector.x);
end    

if (updated.fmp)
    handles.AlphaBeta.update(Data.fmp.alpha, Data.fmp.beta);
    handles.Ruderwege.update(Data.fmp.aileron, Data.fmp.elevator, Data.fmp.rudder, Data.fmp.elevatorTrim);

    set(handles.P_stat, 'String', sprintf('%.1f hPa', Data.fmp.pStat));
    set(handles.P_dyn, 'String', sprintf('%.1f Pa', Data.fmp.pDyn));

    T = 273.15 + Data.fmp.temp; % Außentemperatur in Kelvin
    p_d = 611.213 * exp(17.2799 - (4102.99 / (T - 35.719))) / 100; % Sättigungsdampfdruck von Wasser in hPa
    R_f = 287.058 / (1 - Data.fmp.humi / 100 * p_d / Data.fmp.pStat * (1 - 287.058/461)); % Gaskonstante von feuchter Luft
    rho_0 = 1.225; % Luftdichte am Boden nach ICAO Standardatmosphäre
    rho = Data.fmp.pStat * 100 / R_f / (273.15 + Data.fmp.temp); % wahre Luftdichte
    IAS = sqrt(Data.fmp.pDyn * 2 / rho_0);
    CAS = IAS;
    TAS = CAS * sqrt(rho_0/rho);

    switch(get(handles.IAS, 'UserData'))
        case 0; set(handles.IAS, 'String', sprintf('%.1f km/h', IAS * 3.6));
        case 1; set(handles.IAS, 'String', sprintf('%.1f kt', IAS * 3.6 / 1.852));
        otherwise; set(handles.IAS, 'UserData', 0);
    end
    switch(get(handles.TAS, 'UserData'))
        case 0; set(handles.TAS, 'String', sprintf('%.1f km/h', TAS * 3.6));
        case 1; set(handles.TAS, 'String', sprintf('%.1f kt', TAS * 3.6 / 1.852));
        otherwise; set(handles.TAS, 'UserData', 0);
    end

    set(handles.Temp, 'String', sprintf('%.1f °C', Data.fmp.temp));
    set(handles.Humi, 'String', sprintf('%.1f %%', Data.fmp.humi));

    handles.StartLandeZeit.update(IAS, Data.fmp.header.stamp);
end
% 
% if (updated.NavCompass || updated.NavSol)
%     handles.Kompass.update(NavCompass.MagHeading, NavSol.Azimuth, NavSol.TrueCourse);
% end

if (updated.supply)
    if (numel(Data.supply.voltage) >= 1)
        handles.Spannung.update(Data.supply.voltage(1));
    end
end

if (updated.status)
    switch(Data.status.level)
        case 1
            set(handles.NavStatus, 'String', Data.status.msg, 'BackgroundColor', 'gray');
        case 2
            set(handles.NavStatus, 'String', Data.status.msg, 'BackgroundColor', 'green');
        case 4
            set(handles.NavStatus, 'String', Data.status.msg, 'BackgroundColor', 'yellow');
        case 8
            set(handles.NavStatus, 'String', Data.status.msg, 'BackgroundColor', 'red');
        case 16
            set(handles.NavStatus, 'String', Data.status.msg, 'BackgroundColor', 'red');
    end
end

drawnow;
