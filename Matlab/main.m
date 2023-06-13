% Glowny program sterowania w petli zamknietej
close all
clear all
clc



% inicjalizacja zmiennych
i_right= 0; % prąd prawego silnika
i_left = 0; % prąd lewego silnika

CMD_right=0; % 0 - STOP, 1 - BRAKE, 2 - RUN
CMD_left=0; % 0 - STOP, 1 - BRAKE, 2 - RUN



N=100; % liczba probek 

t = zeros(N, 1);

%**********************************************************************
% Nawiązanie komunikacji z urzadzeniem
s = Open_COMport('COM7');

if( isempty(s) )
    return;
end

[l_m_ss_1st, r_m_ss_1st, arm_Pos, arm_Tens, dt, errs] = parser(s);
%**********************************************************************
tic
for i = 1:N % petla chodzi ok 10Hz
    %**********************************************************************
    % odbieranie danych
    [l_m_ss, r_m_ss, arm_Pos, arm_Tens, dt, errs] = parser(s);
    
    % l_m_ss(1), r_m_ss(1) - pozycja lewego/prawego silnika  [ stopnie ]
    % l_m_ss(2), r_m_ss(2) - prędkość lewego/prawego silnika [ stopnie/s ]
    % l_m_ss(3), r_m_ss(2) - przyspiesznienie lewego/prawego silnika [ stopnie/s^2 ]

    % arm_Pos - pozycja od 0 do 2^15 
    % arm_Tens - sila od 0 do 2^15 (0V-5V )
    
    r_m_ss-r_m_ss_1st
    
    %**********************************************************************
    % tutaj umiescic algorytm sterowania

    CMD_right=2;
    CMD_left=2;

    i_right = 5;
    %% 
    i_left = 0;

 


    %**********************************************************************
    % wysyłanie danych do sterownika
    sendMotorCmds(s, CMD_right,CMD_left,i_right,i_left);
    %**********************************************************************
    t_i = toc; % czas probkowania
    % zapis danych do tablic
    

    t(i) = t_i;
    %**********************************************************************
end


%**********************************************************************
% zatrzymanie silników
CMD_right=0;
CMD_left=0;
sendMotorCmds(s, CMD_right,CMD_left,i_right,i_left);
%**********************************************************************

% zamkniecie komunikacji z urzadzeniem
Close_COMport(s);
