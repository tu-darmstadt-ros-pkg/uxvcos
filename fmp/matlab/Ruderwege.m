classdef Ruderwege
    properties
        handles = struct();
        limitAileron = [-1 1];
        limitElevator = [-1 1];
        limitRudder = [-1 1];
        limitElevatorTrim = [-1 1];
    end
    
    methods
        function obj = Ruderwege(h1, h2, h3)
            obj.handles.AileronElevatorAxis = h1;
            obj.handles.RudderAxis = h2;
            obj.handles.ElevatorTrimAxis = h3;
            
            set([obj.handles.AileronElevatorAxis, obj.handles.RudderAxis, obj.handles.ElevatorTrimAxis], ...
                'Color', 'white', ...
                'Box', 'on', ...
                'XGrid', 'on', 'YGrid', 'on', ...
                'XTickLabel', [], 'YTickLabel', [], ...
                'NextPlot', 'replacechildren' ...
               );
           
            set(obj.handles.AileronElevatorAxis, ...
                'PlotBoxAspectRatio', [1 1 1], ...
                'DataAspectRatio', [1 1 1], ...
                'XLim', obj.limitAileron, ...
                'YLim', obj.limitElevator ...
               );

            set(obj.handles.RudderAxis, ...
                'XLim', obj.limitRudder, ...
                'YLim', [-1 1], ...
                'YTick', [] ...
               );

            set(obj.handles.ElevatorTrimAxis, ...
                'XLim', [-1 1], ...
                'YLim', obj.limitElevatorTrim, ...
                'XTick', [] ...
               );
           
           obj.handles.AileronElevator = plot(obj.handles.AileronElevatorAxis, 0, 0, 'r.', 'MarkerSize', 20);
           obj.handles.Rudder          = plot(obj.handles.RudderAxis,          0, 0, 'r.', 'MarkerSize', 20);
           obj.handles.ElevatorTrim    = plot(obj.handles.ElevatorTrimAxis,    0, 0, 'r.', 'MarkerSize', 20);
end
        
        function update(obj, aileron, elevator, rudder, elevatorTrim)
            set(obj.handles.AileronElevator, 'XData', aileron, 'YData', elevator);
            set(obj.handles.Rudder, 'XData', -rudder);
            set(obj.handles.ElevatorTrim, 'YData', elevatorTrim);
        end
    end
end
