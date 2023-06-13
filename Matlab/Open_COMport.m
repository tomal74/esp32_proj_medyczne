function serial_handle = Open_COMport(serialPortName)
    delete(instrfind({'Port'},{serialPortName}));
    allCom = serialportlist("all");
    avaliableCom = serialportlist("available");
    
    if(isempty(allCom))
        disp("No COM ports available: ")
        serial_handle = [];
        return
    else   
        disp("All COM ports: ")
        disp(allCom)

        disp("Available COM ports: ")
        disp(avaliableCom)
    end

    % Konfiguracja parametrów portu szeregowego
    baudRate = 115200; % Szybkość transmisji w bodach
    dataBits = 8; % Liczba bitów danych
    stopBits = 1; % Liczba bitów stopu

    % Inicjalizacja obiektu portu szeregowego
    serial_handle = serialport(serialPortName, baudRate, 'DataBits', dataBits, 'StopBits', stopBits);
    serial_handle.flush();

    
end