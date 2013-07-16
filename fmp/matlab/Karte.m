classdef Karte < handle
    properties
        handles = struct();
        range = 1000;
        trace = zeros(0,3);
        tracetime = 300;
    end
    
    methods
        function obj = Karte(h)
            obj.handles.axis = h;

            set(obj.handles.axis, ...
                'PlotBoxAspectRatio', [1 1 1], ...
                'DataAspectRatio', [1 1 1], ...
                'Color', 'white', ...
                'XLim', [-obj.range +obj.range], ...
                'YLim', [-obj.range +obj.range], ...
                'Box', 'on', 'XTick', [], 'YTick', [], ...
                'NextPlot', 'replacechildren' ...
               );
           
           obj.handles.trace = plot(obj.handles.axis, nan, nan, 'b.', 'MarkerSize', 10);
        end
        
        function update(obj, time, latitude, longitude)
            latitude = latitude * pi/180;
            longitude = longitude * pi/180;
            RLat = 6371000;
            RLon = RLat * cos(latitude);
            if (~isempty(obj.trace) && time < obj.trace(end,1) + .5)
                return;
            end
            
            if (isnan(latitude) || isnan(longitude))
                return;
            end
            
            obj.trace(end+1,:) = [time, latitude * RLat, longitude * RLon];
            obj.trace = obj.trace(obj.trace(:,1) > time - obj.tracetime, :);
            
            if (isempty(obj.trace)); return; end
            set(obj.handles.axis, ...
                'XLim', [obj.trace(end,3)-obj.range obj.trace(end,3)+obj.range], ...
                'YLim', [obj.trace(end,2)-obj.range obj.trace(end,2)+obj.range]);
            set(obj.handles.trace, 'XData', obj.trace(:,3), 'YData', obj.trace(:,2));
        end
        
        function reset(obj)
            obj.trace = zeros(0,3);
        end
    end
end
