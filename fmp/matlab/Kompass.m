classdef Kompass
    properties
        handles = struct();
        CircleRadius = 0.8;
        VectorLength = 0.7;
    end
    
    methods
        function obj = Kompass(h)
            obj.handles.axis = h;
            
            set(obj.handles.axis, ...
                'PlotBoxAspectRatio', [1 1 1], ...
                'DataAspectRatio', [1 1 1], ...
                'XLim', [-1 1], ...
                'YLim', [-1 1], ...
                'Visible', 'off', ...
                'NextPlot', 'replacechildren' ...
               );
           
            rectangle('Position', [-obj.CircleRadius -obj.CircleRadius 2*obj.CircleRadius 2*obj.CircleRadius], 'Curvature', [1 1], 'LineWidth', 2, 'FaceColor', 'white', 'Parent', obj.handles.axis);
            ticks = [0:30:330]*pi/180;
            for t = ticks
                line(sin(t)*[obj.CircleRadius-0.05 obj.CircleRadius], cos(t)*[obj.CircleRadius-0.05 obj.CircleRadius], 'Color', 'black', 'LineWidth', 2, 'Parent', obj.handles.axis);
            end
            
            text(0,  1, 'N', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top',    'Parent', obj.handles.axis);
            text(1,  0, 'O', 'FontWeight', 'bold', 'HorizontalAlignment', 'right',  'VerticalAlignment', 'middle', 'Parent', obj.handles.axis);
            text(0, -1, 'S', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Parent', obj.handles.axis);
            text(-1, 0, 'W', 'FontWeight', 'bold', 'HorizontalAlignment', 'left',   'VerticalAlignment', 'middle', 'Parent', obj.handles.axis);
            
            obj.handles.mwSK = line([0 0], [0 obj.VectorLength], 'Color', 'r', 'LineWidth', 1, 'Parent', obj.handles.axis);
            obj.handles.rwSK = line([0 0], [0 obj.VectorLength], 'Color', 'r', 'LineWidth', 2, 'Parent', obj.handles.axis);
            obj.handles.rwK  = line([0 0], [0 obj.VectorLength], 'Color', 'b', 'LineWidth', 2, 'Parent', obj.handles.axis);
        end
        
        function update(obj, mwSK, rwSK, rwK)
            set(obj.handles.mwSK, 'XData', [0 sin(mwSK)] * obj.VectorLength, 'YData', [0 cos(mwSK)] * obj.VectorLength);
            set(obj.handles.rwSK, 'XData', [0 sin(rwSK)] * obj.VectorLength, 'YData', [0 cos(rwSK)] * obj.VectorLength);
            set(obj.handles.rwK , 'XData', [0 sin(rwK)]  * obj.VectorLength, 'YData', [0 cos(rwK)]  * obj.VectorLength);
        end
    end
end
