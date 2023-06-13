function sendMotorCmds(serial_handle, CMD_right, CMD_left, i_right, i_left)
%SENDMOTORCMDS Summary of this function goes here
%   Detailed explanation goes here
        persistent cnt
        if isempty(cnt)
            cnt = 0x00;
        end 
        % Tworzenie ramki danych
        crc = 0x00;
        startByte = [0xaa 0x55]; % Bajt początkowy ramki 
        
        len= [0x00 0x14]; % Długość ramki danych
        type= 0x07; % Typ ramki

        dataFrame = [uint8(cnt),typecast(uint8(CMD_right),'uint8'),typecast(uint8(CMD_left),'uint8'),typecast(single(i_right),'uint8'),typecast(single(i_left),'uint8'),0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]; % Dane do wysłania
        cnt = cnt + 0x01;
         for i = 1:length(dataFrame)
             crc = bitxor(crc, uint8( dataFrame(i) ));
            % crc = bitxor(crc, bitshift(dataFrame(i), 8)); % XOR z bajtem danych przesuniętym o 8 bitów w lewo
         end
        
        
      %// 0xAA 0x55 0x00 0x02 0x1F 0x00 0x01 0x01 0xXX
        % aa    55  lenH lenL type    DATA------...XOR
            
        % Składanie ramki danych
        frame = [startByte, len,type, dataFrame, crc];
        
        % Wysyłanie ramki
        write(serial_handle, frame, "uint8");
       
end

