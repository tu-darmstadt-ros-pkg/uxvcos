%% Parameter des Zustandautomaten

%zus�tzlich werden ben�tigt: 
LGH =.15;               % [m] hoehe des landegestells

%am boden
SchubRun = .2;           % [0..1]real [0..10] sim Schubbefehl zum testen der Motoren
tAnlauf = 2.5;            % [s] Anlaufzeit

%starten
SchubStart = 0.65;       % [0..1]real [0..10] sim Maximalschub zum starten
dSchubStart = .01;       % inkrement pro schritt
dtStart = .2;      % "
hFlug = 1;             % hoehe �ber boden bei �bergang von/zu freiem flug
zpStart = .1;         % geschwindigkeit beim starten

%landen
twc = .1;            % zeit zwichen windmessungen beim landen
dhZulSinkflug = .05;  % zul�ssige h�hendifferenz bei sinkflug vor n�chsten schritt
hLanden = LGH*4;       % hoehe �ber boden bei �bergang von sinkflug zu landen
dhSinkflug = .05;      % inkrement bei sinkflug
zpLanden = .05;        % geschwindigkeitsvorgabe beim landen
dSchubLanden= .05;      % inkrement vom Schub beim landen
dtLanden=.1;

% ladestandeinfluss
Bstart=5;         % [min] restflugzeit vor start
Bkrit=1;          % [min] restflugzeit triggered Notlandung
Bsafe=3;          % [min] begin der erniedrigung der kriterien bei Bsafe minuten restflugzeit

%lagetol und windcheck
zul_neig=.15;                       % [rad] zul�ssige bodenneigung
zul_win=.15;
%zul_win=zulwin(); % [rad] daraus resultierende grenzen f�r roll und yaw vor landung
zul_dp=.15;                         % [m] zul�ssige positionstoleranz
zul_dwmax=0.8;                      % [m/s] maximal zul�ssige ver�nderung der windgeschwindikeit �ber die letzten 2 sec vor landung
zul_dmotor=25;                      % [rpm] maximal zul�ssige abweichung der motordrehzahlen vor start
zul_w=2.5;                          % [m/s] schwelle unterhalb derer b�igkeit ignoriert wird