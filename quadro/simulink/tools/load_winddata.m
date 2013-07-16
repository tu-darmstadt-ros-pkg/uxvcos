winddata = importdata('110826wind.txt');
T_wind_sample = 5;

%rad_wind = deg2rad( winddata(:,1));% license err
rad_wind = winddata(:,1)./360.*2*3.1415926;
v_wind = [winddata(:,2) winddata(:,2)].* [ cos(rad_wind) sin(rad_wind)];
data_points = length(v_wind);
T_wind  = linspace(0, (data_points-1)*T_wind_sample, data_points);
T_wind_fine  = linspace(0, (data_points-1)*T_wind_sample,...
                 T_wind_sample*(data_points-1));
v_wind_x_fine = spline(T_wind, v_wind(:,1)',T_wind_fine);
v_wind_y_fine = spline(T_wind, v_wind(:,2)',T_wind_fine);
v_wind_fine = [ v_wind_x_fine' v_wind_y_fine'];


show_wind = 0;
if (show_wind)
    example_points = 24;
    example_points_fine = (example_points-1)*T_wind_sample;
    T_example  = linspace(0, (example_points-1)*T_wind_sample, example_points);
    T_example_fine  = linspace(0, (example_points-1)*T_wind_sample,...
                      T_wind_sample*(example_points-1));
    figure
    plot(T_example, v_wind(1:example_points, 1))
    figure
    plot(T_example_fine, v_wind_fine(1:example_points_fine, 1))
end
