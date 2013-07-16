function output = reducevector(signal)

datapoints = 1000;

step = floor(length(signal)/datapoints);
if (step>1)
    output = downsamplevector(signal, step);
else 
    output = signal;
end

end
