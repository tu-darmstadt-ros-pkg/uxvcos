classdef Logging < handle
    properties
        meta = struct();
    end
    
    properties (Access = private)
        file = 0
        metafile = []
        starttime = 0
        timestamp = 0
        pfadname = []
        logdir = ['..' filesep 'log' filesep]
    end
    
    methods
        function obj = Logging(meta, pfadname)
            obj.meta = meta;
            
            if (nargin < 1)
                pfadname    = [obj.logdir datestr(now, 29) '_' meta.Name];
                pfadname    = strrep(pfadname, ' ', '_');
            end

            obj.pfadname = pfadname;
            %obj.start();
        end
        
        function delete(obj)
            obj.stop();
        end
        
        function start(obj, dateiname)
            if (obj.file ~= 0); return; end
            
            global Data;
            obj.meta.Start = Data.timestamp;
            
            if (nargin < 2); dateiname = []; end
            if (isempty(dateiname))
                dateiname   = [datestr(obj.meta.Start/86400 + 719529, 'HHMMSS') '_' obj.meta.Versuch];
                dateiname   = strrep(dateiname, ' ', '_');
            end
            
            [status, message, messageid] = mkdir(obj.pfadname);
            if (~status && ~strcmp(messageid, 'MATLAB:MKDIR:DirectoryExists'))
                error(message);
                return;
            end
            
            txtfile = fopen([obj.pfadname filesep 'Beschreibung.txt'], 'w');
            if (txtfile ~= -1)
                fn = fieldnames(obj.meta);
                for i = 1:size(fn)
                    fprintf(txtfile, '%s: %s\n', fn{i}, num2str(obj.meta.(fn{i})'));
                end
            end
            fclose(txtfile);
            
            obj.metafile = [obj.pfadname filesep dateiname '.mat'];
            Meta = obj.meta;
            Meta.Datum = datestr(obj.meta.Start/86400 + 719529, 'dd.mm.yyyy');
            Meta.Start = datestr(obj.meta.Start/86400 + 719529, 'HH:MM:SS');
            save(obj.metafile, '-struct', 'Meta');

            [obj.file, message] = fopen([obj.pfadname filesep dateiname '.txt'], 'w');
            if (obj.file == -1)
                obj.file = 0;
                error(message);
                return;
            end
            
            obj.starttime = 0;
            fprintf(obj.file, 'time roll pitch heading latitude longitude altitude v_north v_east v_down track alpha beta p_stat p_dyn temperature humidity aileron elevator rudder elevatortrim\n');
            
            %obj.listener = obj.connector.addlistener('Update', @(src, event) obj.log(src.data));
        end
        
        function stop(obj)
            %if (obj.listener ~= 0); delete(obj.listener); end
            %obj.listener = 0;
            if (obj.file ~= 0); fclose(obj.file); end
            obj.file = 0;
            
            global Data;
            obj.meta.Stop = Data.timestamp;
            Meta = obj.meta;
            Meta.Stop = datestr(obj.meta.Stop/86400 + 719529, 'HH:MM:SS');
            save(obj.metafile, '-struct', 'Meta');
        end
        
        function log(obj, source, event, updated)
            if (obj.file == 0); return; end
            global Data;

            %if (Data.timestamp > obj.timestamp)
                obj.timestamp = Data.timestamp;
            %end
            
            if (obj.starttime == 0)
                obj.starttime = obj.timestamp;
            end

            [Data.euler(1) Data.euler(2) Data.euler(3)] = quat2angle([Data.state.pose.pose.orientation.w Data.state.pose.pose.orientation.x Data.state.pose.pose.orientation.y Data.state.pose.pose.orientation.z]);
            Data.euler = Data.euler .* [-1 -1 1];
            Data.course = -atan2(Data.state.twist.twist.linear.y, Data.state.twist.twist.linear.x);

            fprintf(obj.file, '%10.3f %f %f %f %.16g %.16g %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n', obj.timestamp, ...
                Data.euler(3), Data.euler(2), Data.euler(1), ...
                Data.global.latitude, Data.global.longitude, Data.global.altitude, ...
                Data.state.twist.twist.linear.x, -Data.state.twist.twist.linear.y, -Data.state.twist.twist.linear.z, ...
                Data.course, ...
                Data.fmp.alpha, Data.fmp.beta, ...
                Data.fmp.pStat, Data.fmp.pDyn, ...
                Data.fmp.temp, Data.fmp.humi, ...
                Data.fmp.aileron, Data.fmp.elevator, Data.fmp.rudder, Data.fmp.elevatorTrim);
        end
    end
end

