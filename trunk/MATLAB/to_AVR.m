function to_AVR(theport, packet_type, byte_array) 
% Send a n byte message to the AVR (as well as start and stop bytes)
% where n is the size of byte_array (maximum 32 bytes)
% Packet needs to be less than or equal to 32 bytes long, data only (u08)
% Data is supplied in a packet with a single column (not row) hence it is
% transposed by the code below.
% Catch exceptions and ignore them (this shouldn't happen much after
% timeout problem was fixed by downloading patch for fwrite)
% Mark Whitty
% 02/07/07
% This protocol uses 10 as the start byte and includes a checksum but no
% stop byte. 

global MAX_PACKET_SIZE;
global START_BYTE;

MAX_PACKET_SIZE = 32;
START_BYTE = 10;

uint16 checksum;
num_bytes = size(byte_array, 1);
if(num_bytes > 32) 
    'Too many bytes sent to to_AVR - maximum of 32'
    num_bytes
    return
end
checksum = packet_type+num_bytes;
for i = 1:num_bytes
    checksum = checksum + uint16(byte_array(i));
end
checksum = uint8(bitand(checksum, 255));
output = [START_BYTE, packet_type, num_bytes, byte_array', checksum];

try  
    fwrite(theport,output);
catch
   'Caught exception while sending whole message'
end


end

