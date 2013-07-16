classdef Horizont < handle
    properties
        handles = struct();
        viewAngle = 50 * pi/180;
    end
    
    methods
        function obj = Horizont(h)
            obj.handles.axis = h;

            set(obj.handles.axis, ...
                'PlotBoxAspectRatio', [1 1 1], ...
                'DataAspectRatio', [1 1 1], ...
                'Color',[0.678431391716003 0.921568632125854 1], ...
                'Box', 'on', 'XTick', [], 'YTick', [], ...
                'XLimMode', 'manual', 'XLim', [-1 1], ...
                'YLimMode', 'manual', 'YLim', [-1 1], ...
                'NextPlot', 'add' ...
               );
           
            obj.handles.Horizon = hgtransform('Parent', obj.handles.axis);
            obj.handles.Ground = patch([-2 2 2 -2], [0 0 -2 -2], [0.682352960109711 0.466666668653488 0], 'Clipping', 'on', 'LineStyle', 'none', 'Parent', obj.handles.Horizon);

            % Fadenkreuz
            line([-0.5 -0.2; -0.2 -0.2], [0 0; 0 -0.05], 'LineWidth', 1, 'Color', 'black', 'Parent', obj.handles.axis);
            line([ 0.5  0.2;  0.2  0.2], [0 0; 0 -0.05], 'LineWidth', 1, 'Color', 'black', 'Parent', obj.handles.axis);
            line(0, 0, 'Marker', '.', 'MarkerSize', 20, 'Color', 'black', 'Marker', '.', 'Parent', obj.handles.axis);
            
            % Pitch Skala
            majorticks = [-30:10:30];
            minorticks = [-25:10:25];
            for t = majorticks
                line([-0.3 0.3], [t t]*pi/180/obj.viewAngle, 'LineWidth', 1, 'Color', 'black', 'Parent', obj.handles.Horizon);
                text(-0.4, t*pi/180/obj.viewAngle, num2str(t), 'FontSize', 6, 'HorizontalAlignment', 'right', 'Clipping', 'on', 'Parent', obj.handles.Horizon);
                text( 0.4, t*pi/180/obj.viewAngle, num2str(t), 'FontSize', 6, 'HorizontalAlignment', 'left', 'Clipping', 'on', 'Parent', obj.handles.Horizon);
            end
            for t = minorticks
                line([-0.1 0.1], [t t]*pi/180/obj.viewAngle, 'LineWidth', 1, 'Color', 'black', 'Parent', obj.handles.Horizon);
            end
            
            % Roll Skala
            patch([0 0.02 -0.02], [0.9 0.85 0.85], 'black', 'Parent', obj.handles.axis);
            obj.handles.RollIndicator = hgtransform('Parent', obj.handles.axis);
            %t = [-60:60]*pi/180;
            %line(sin(t)*0.9*obj.viewAngle, cos(t)*0.9*obj.viewAngle, 'LineWidth', 2, 'Color', 'black', 'Parent', obj.handles.RollIndicator);
            majorticks = [-60 -45 -30 30 45 60];
            minorticks = [-20 -10 10 20];
            for t = majorticks
                line(sin(t*pi/180)*[0.9 0.95], cos(t*pi/180)*[0.9 0.95], 'LineWidth', 2, 'Color', 'black', 'Parent', obj.handles.RollIndicator);
                %text(-0.4*obj.viewAngle, t*pi/180, num2str(t), 'HorizontalAlignment', 'right', 'Parent', obj.handles.Horizon);
            end
            for t = minorticks
                line(sin(t*pi/180)*[0.9 0.93], cos(t*pi/180)*[0.9 0.95], 'LineWidth', 1, 'Color', 'black', 'Parent', obj.handles.RollIndicator);
            end
            patch([0 0.02 -0.02], [0.9 0.95 0.95], 'black', 'Parent', obj.handles.RollIndicator);
        end
        
        function update(obj, roll, pitch)
            if isnan(roll) || isnan(pitch); return; end
            set(obj.handles.Horizon, 'Matrix', makehgtform('zrotate', roll, 'translate', [0 -pitch/obj.viewAngle 0]));
            set(obj.handles.RollIndicator, 'Matrix', makehgtform('zrotate', roll));
        end
    end
end

