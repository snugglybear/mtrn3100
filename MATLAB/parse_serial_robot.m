% parse_serial_root extracts and parses data from an input buffer of uint8
% values received from a control board on the robot communicating via a
% serial linkage. It can only handle a single packet type of variable
% length. Note the length is also controlled by the size of the input
% buffer.
% Inputs: fromAVR - a row vector containing the serial buffer contents
%           numbytesread - number of bytes in this array
% Outputs: Are specific to the application. 
% IR_1 is the distance (in m) detected by the left
% IR sensor , IR_2 is the distance (in m) detected by the right IR sensor,
% US_range_mm is the distance (in mm) detected by the ultrasonic sensor.
%
% MTRN9222 Artificially Intelligent Machine
% Code by Mark Whitty <m.whitty@student.unsw.edu.au>
%
% 080410 Version 0.1
function [ID, direction, motorA, motorB, servo, US_range_mm, IR, parse_status] = parse_serial_robot(fromAVR, numbytesread);

% You should not need to modify this function - do so at your own peril.

global START_BYTE;  % Global variable declarations must be included in each file they are used.
global UGVID;

for i = 1:(numbytesread-3)   % Note that a message must contain at least 4 bytes due to the packet format used
    if (fromAVR(i) == START_BYTE)
        % found suspected first byte in message
        expected_packet_length = fromAVR(i+2);
        % Ensure whole packet could fit in the buffer
        if((expected_packet_length + (i+2) + 1) > numbytesread)
            'whole packet isn''t in the buffer'
            break;
        end
        expected_packet_id = fromAVR(i+1);
        % Check that correct packet ID is being received
        if(expected_packet_id ~= 1+UGVID)
            'packet ID doesn''t exist'
            break;
        end
        checksum = 0;
        for j = (i+1):(i+expected_packet_length+2)
            % Calculate the actual checksum
            checksum = checksum + fromAVR(j);
            checksum = mod(checksum, 256);   % ensure uint8 value is wrapped around on each addition for correct calculation of the checksum
                                            % not just by modulo on the final addition (which doesn't cater for multiple overflows)      
        end
        if(checksum ~= fromAVR(i+expected_packet_length+3))
            % checksum doesn't match
            'checksum doesn''t match'
            break;
        end
        % the packet has been recognised as matching the desired format,
        % now extract the data
        ID = fromAVR(i+1)-1;
        if ID ~= UGVID          %Wrong ID, received message from other UGV
            parse_status = 1;   % return error flag
            direction = -1;
            motorA = -1;
            motorB = -1;
            servo = -1;
            US_range_mm = -1;
            IR = -1;
            return;
        end
        direction = fromAVR(i+3);
        motorA = fromAVR(i+4);
        motorB = fromAVR(i+5);
        servo = fromAVR(i+6);
        us_low = fromAVR(i+7);
        us_high = fromAVR(i+8);
        US_range_mm = (bitor(us_low, bitshift(us_high, 8))*0.592+9.414)*1.6;
        IR_low = fromAVR(i+9);
        IR_high = fromAVR(i+10);
        IR = bitor(IR_low, bitshift(IR_high, 8));
        if(IR == 0)
            IR = inf;
        else
            IR = 2 * 29.4 / IR;
        end
        if(IR > 2) 
            IR = inf;
        end
        parse_status = 0;
        return;
    end
end

% No START_BYTEs in message, return error flag
parse_status = 1;   % return error flag
ID = -1;
direction = -1;
motorA = -1;
motorB = -1;
servo = -1;
US_range_mm = -1;
IR = -1;
