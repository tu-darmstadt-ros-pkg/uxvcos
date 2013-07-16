%% Arbeitspunkt
U = [U0 U0 U0 U0];                                    %[V]        Stellvektor im Arbeitspunkt (Schwebeflug)

% Skalierungsfaktoren
PWM2N  = 0.0325; 
PWM2NM = 4.6350e-04; 

% Vorsteuerung für Drehzahlberechnunng aus Sollkraft
c0 =  0.0137;
c1 = -0.0117;

%% Parameter für Regler

% Hoehe
Kp_h = 40*13/100;
Ki_h = 20*13/100;
Kd_h = 40*13/100;

% Position %dauerschwingung bei Kpkrit=18.75; Tkrit=4;
Kp_p = 9*13/100;
Ki_p = 0*13/100;
Kd_p = 12.5*13/100;
Kges_p =1;

% Geschwindigkeit
Kr_w=0.2;        %0.2;                
Kp_w=0.6;          %0.6;                      
Ki_w=0.045;      %0.045;                                       

% Nicken und Rollen
Kr_g=PWM2N;  
Kp_g=60*13/100;
Ki_g=30*13/100;
Kd_g=30*13/100;

% GIERREGLER
Kr_y=PWM2NM;
Kp_y=150; 
Ki_y=0;
Kd_y=60;