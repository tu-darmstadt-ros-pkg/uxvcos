% plot the values with the name being the struct
function retval = plot_struct(name, time, varargin )

figure('Name', name);
horizontal = length(varargin);
lengtharray= zeros(1,horizontal);


for ii = 1:horizontal
	struct_names = fieldnames(varargin{ii});
	lengtharray(1,ii) = length(struct_names);
end

vertical = max(lengtharray);

for ii = 1:length(varargin)
	struct_names = fieldnames(varargin{ii});
	length_struct = length(struct_names);
	for jj=1:length_struct
		length_of_time = length(time);
 		output = varargin{ii}.(struct_names{jj});
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

		y = y(1:length_of_time);
		subplot(vertical, horizontal,horizontal*(jj-1)+ii)
		plot(time, y);
		ylabel(struct_names{jj});
	end	
end

retval = 0;

end
