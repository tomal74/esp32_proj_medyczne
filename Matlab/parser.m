function [l_m_ss, r_m_ss, arm_Pos, arm_Tens, data_type, data_errs] = parser(s)
%PARSER Summary of this function goes here
%   Detailed explanation goes here
BUFLEN = 128;
persistent ss_state ss_len ss_type ss_idx ...
ss_checksum ss_buf check_xor ss_c rs232errors
if isempty(ss_state)
    ss_state = 0;
    ss_len = 0;
    ss_type = 0x00;
    ss_idx = 0;
    ss_buf = zeros(1, BUFLEN);
    ss_checksum = 0x00;
    check_xor = 0;
    ss_c = 0x00;
    rs232errors = 0;
    tic
end 


%s.NumBytesAvailable()
parsed_data = [];

ss_cc = read(s, 5+33+1, "uint8");
ss_i=0;
while( ss_i~=size(ss_cc,2))
    ss_i=ss_i+1;
    ss_c = ss_cc(ss_i);

    switch ss_state
    case 0
        if(ss_c==0xAA) 
            ss_state = 1;
        end
    case 1
      if(ss_c==0x55)
          ss_state = 2;
      else
      
        if(ss_c~=0xAA)
          ss_state=0;
          rs232errors = rs232errors + 1;
          s.flush();
          disp('parser reset...')
          disp('all err ')
          disp(rs232errors)
        end

      end      
    case 2
          ss_len = bitshift(uint16(ss_c), 8, "uint16");
          ss_state = 3;
    case 3
          ss_len = bitor(uint16(ss_len), uint16(ss_c), 'uint16');  %  |= ss_c;
          if(ss_len > BUFLEN) 
            ss_len = 0;
            ss_state = 0;
            s.flush();
          else 
            ss_state = 4;
          end
            
    case 4
          ss_type = ss_c;
          ss_state = 5;
          ss_idx = 1;
    case 5
          ss_buf(ss_idx) = ss_c;
          ss_idx = ss_idx + 1;
          if(ss_idx == ss_len+1) 
              ss_state = 6;
          end
          
    case 6
          ss_checksum = uint8(ss_c);
          check_xor = 0;

          for(i=1:ss_len) 
              check_xor = bitxor(check_xor, uint8(ss_buf(i) ) ); %^= ss_buf[i];
          end

          if(check_xor == ss_checksum)
            
            switch(ss_type)
                case 0x08 % encoder type
                
                u8frameCNT = typecast(uint8(ss_buf(1)), 'uint8');

                r_motor_degPos = typecast(uint8(ss_buf(2:5)), 'single');
                l_motor_degPos = typecast(uint8(ss_buf(6:9)), 'single');

                r_motor_vel = typecast(uint8(ss_buf(10:13)), 'single');
                l_motor_vel = typecast(uint8(ss_buf(14:17)), 'single');

                r_motor_acc = typecast(uint8(ss_buf(18:21)), 'single');
                l_motor_acc = typecast(uint8(ss_buf(22:25)), 'single');

                arm_Pos = typecast(uint8(ss_buf(26:27)), 'uint16');
               
                arm_Tens = typecast(uint8(ss_buf(28:29)), 'uint16');

                l_m_ss=[l_motor_degPos,l_motor_vel,l_motor_acc];
                r_m_ss=[r_motor_degPos,r_motor_vel,r_motor_acc];


                otherwise
                    disp("Unsupported type of frame - 0x" + num2str(dec2hex(ss_type)))
                    parsed_data = [];
            end
                
          else  % checksum err
              disp("Serial - i've frame - but incorrect")
              rs232errors = rs232errors + 1;
              s.flush();
          end
            
          ss_state = 0;
    otherwise
          ss_state = 0;
        end
    end
data_errs = rs232errors;
data_type = ss_type;
end



