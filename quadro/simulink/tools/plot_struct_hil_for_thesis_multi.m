% plot the values with the name being the struct
function plot_struct_hil_for_thesis_multi(name, time, varargin )

time_short = preprocess_hil(time);

%figure('Name', name);
horizontal = length(varargin);
lengtharray= zeros(1,horizontal);
colors = hsv(5);


for ii = 1:horizontal
	struct_names = fieldnames(varargin{ii});
	lengtharray(1,ii) = length(struct_names);
end

length_struct_max = 0;
for ii = 1:length(varargin)
	struct_names = fieldnames(varargin{ii});
	length_struct_max = max(length_struct_max, length(struct_names));
end

vertical = max(lengtharray);
cell_of_figures = cell(length_struct_max);
cell_of_names = cell(length_struct_max);


for jj=1:length_struct_max
    cell_of_figures{jj} = figure;
    set(cell_of_figures{jj},'Visible','off');
    cell_of_names{jj} = '';
end

for ii = 1:length(varargin)
	struct_names = fieldnames(varargin{ii});
	length_struct = length(struct_names);
	for jj=1:length_struct
		length_of_time = length(time);
 		output =  varargin{ii}.(struct_names{jj});

		% size_output = size(output)
		% size_time = size(time)
		length_of_y = length(output);
		% dirty hack to ensure it plots
		if (length_of_y <= length_of_time)
			y = zeros(length_of_time,size(output,2));
			y(1:size(output,1),:) = output;
		else
			y = output(1:length_of_time,:);
		end

		y = preprocess_hil(y(1:length_of_time));
        y_short = preprocess_hil(y);
		%subplot(vertical, horizontal,horizontal*(jj-1)+ii)
        hold on;
        figure(cell_of_figures{jj});
        set(gcf,'Visible','off');
		plot(time_short, y_short, 'color', colors(ii,:));
		ylabel(struct_names{jj});
        cell_of_names{jj} = strcat(cell_of_names{jj}, '_' , struct_names{jj});
	end	
end

for jj=1:length_struct_max
    figure(cell_of_figures{jj});
    matlab2tikz([name  cell_of_names{jj} '.tikz'])
    saveas(gcf, [name  cell_of_names{jj} '.jpg'])
    close; 
end


end
