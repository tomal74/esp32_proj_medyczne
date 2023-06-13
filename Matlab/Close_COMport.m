function Close_COMport(serial_handle)
%CLOSE_COMPORT Summary of this function goes here
%   Detailed explanation goes here
% Zamknięcie portu szeregowego
%close(serial_handle);

% Czyszczenie obiektu portu szeregowego
%delete(serial_handle);
clear serial_handle;
clear sendMotorCmds
end




