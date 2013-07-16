function output = preprocess_sim(signal)
Ts = 0.001;

cutted = signal((1+40/Ts):end);

output = reducevector(cutted);
end
