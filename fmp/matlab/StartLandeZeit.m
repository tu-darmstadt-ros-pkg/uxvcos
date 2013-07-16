classdef StartLandeZeit < handle
    properties
        handles = struct();
        thresholdStart = 50 / 3.6;
        thresholdLandung = 40 / 3.6;
        startzeit = NaN;
        landezeit = NaN;
    end
    
    methods
        function obj = StartLandeZeit(hStart, hFlugzeit, hLandung)
            obj.handles.Start = hStart;
            obj.handles.Flugzeit = hFlugzeit;
            obj.handles.Landung = hLandung;
            
            set(obj.handles.Start, 'String', '');
            set(obj.handles.Flugzeit, 'String', '');
            set(obj.handles.Landung, 'String', '');
        end
        
        function obj = update(obj, ias, time)
            if isnan(ias); return; end
            if isnan(obj.startzeit) && ias > obj.thresholdStart
                obj.startzeit = time;
                set(obj.handles.Start, 'String', sprintf('%02d:%02d', floor(mod(time / 3600, 24)), floor(mod(time / 60, 60))));
            end
            if ~isnan(obj.startzeit) && isnan(obj.landezeit)
                flugzeit = time - obj.startzeit;
                set(obj.handles.Flugzeit, 'String', sprintf('%02d:%02d', floor(mod(flugzeit / 3600, 24)), floor(mod(flugzeit / 60, 60))));
            end
            if ~isnan(obj.startzeit) && ias < obj.thresholdLandung
                obj.landezeit = time;
                set(obj.handles.Landung, 'String', sprintf('%02d:%02d', floor(mod(time / 3600, 24)), floor(mod(time / 60, 60))));
            end
        end
    end
end
