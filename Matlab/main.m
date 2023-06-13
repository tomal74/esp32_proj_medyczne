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


% pl = [];
% time = [];
%
% om = 0;
% kp = 0.0001;
% ki = 0.009;
% kd = 0.004;
% u = 0;
% om_s = 700;
% errp = 0;
% errprev = 0;
% errd = 0;
% omV = [];
%
% pd = [];

%
% for(i=0:0.01:2*pi)
%     while(1)
%         y = 0.5*sin(i*3);
%         i_left = y;
%         i_right = u;
%
%         %sendMotorCmds(s, CMD_right,CMD_left,i_right,i_left); % wysyłanie
%         %danych do sterownika
%
%         sendMotorCmds(s, CMD_right,CMD_left,i_right,i_left); % Zatrzymanie silników
%
%         if(~isempty(pd))
%             pl = [pl pd(1)];
%             time = [time toc];
%             %plot(time, pl)
%             drawnow limitrate
%             grid on
%
%             if(size(pl, 2) > 1)
%                 dt = 0.03;%time(end) - time(end-1);
%                 dd = pl(end) - pl(end-1);
%
%                 om = dd/dt;
%
%                 err = om_s - om;
%                 errp = errp + err*dt;
%                 errd = (err - errprev) / dt;
%                 u = kp*dt*err + ki*dt*(errp); %+ kd*(errd)
%                 %u = 0.006*dt*(errd)
%                 errprev = err;
%
%                 omV = [omV om];
%                 plot(time(2:end), omV)
%                 %ylim([-30, 30])
%                 grid on
%             end
%         end
%
%         %pause(0.01)
%
%         toc
%         tic
%
%
%
%         %pause(0.1)
%     end
% end
%
% %sendMotorCmds(s, 2,0,2,0); % wysyłanie danych do sterownika
%
% Close_COMport(s);
