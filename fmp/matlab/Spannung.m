classdef Spannung < handle
    properties
        handles = struct();
        limits = [10.0 12.5];
        colors = struct('threshold', [ 10.5, 11.5 ], 'color', {{ 'red', 'yellow', 'green' }});
        value = NaN;
    end
    
    methods
        function obj = Spannung(h)
            obj.handles.Axes = h;
            
            set(obj.handles.Axes, ...
                'Color', [0.8 0.8 0.8], ...
                'Box', 'on', 'XTick', [], 'YTick', [], ...
                'XGrid', 'on', 'YGrid', 'on', ...
                'XTickLabel', [], 'YTickLabel', [], ...
                'NextPlot', 'replacechildren', ...
                'XLimMode', 'manual', 'XLim', obj.limits, ...
                'YLimMode', 'manual', 'YLim', [0 1] ...
               );
           
           obj.handles.Bar  = rectangle('Position', [obj.limits(1), 0.0, obj.limits(2) - obj.limits(1), 1.0], 'FaceColor', [0.5, 0.5, 0.5], 'LineStyle', 'none');
           obj.handles.Text = text(0.4, 0.5, 'xxx V', 'Units', 'normalized', 'FontWeight', 'bold');
        end
        
        function obj = update(obj, voltage)
            if ~isnan(obj.value)
                obj.value = 0.1 * voltage + 0.9 * obj.value;
            else
                obj.value = voltage;
            end
            
            if ~isnan(obj.value)
                p = get(obj.handles.Bar, 'Position');
                p(3) = obj.value - obj.limits(1);
                set(obj.handles.Bar, 'Position', p);
            end
            set(obj.handles.Text, 'String', [num2str(obj.value, 3) ' V']);
           
            color_index = 1;
            for i = 1:length(obj.colors.threshold)
                if obj.value > obj.colors.threshold(i)
                    color_index = i+1;
                end
            end
            set(obj.handles.Bar, 'FaceColor', obj.colors.color{color_index});
        end
    end
end
