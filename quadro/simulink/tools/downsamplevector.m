function output = downsamplevector(signal, everyn)
lengthofoutput = floor(length(signal)/everyn);
output = zeros(lengthofoutput,1);

for ii=1:lengthofoutput
    output(ii) = signal((ii-1)*everyn + 1);
end

end
