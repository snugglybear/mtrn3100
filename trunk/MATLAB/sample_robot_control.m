% sample_robot_control contains sample code to control a differentially
% steered robot through an XBee wireless connection. This code calls the
% run_robot function and you should modify this code to control your robot.
% Various initialisation parameters are set up in this code which should
% only be changed if you know what you are doing. 
%
%
% MTRN3100 Artificially Intelligent Machine
% Code by Mark Whitty<m.whitty@student.unsw.edu.au>
% and Edward Mak <lin-chi@student.unsw.edu.au>
% If you find any problems or bugs in this sample code, please e-mail the
% authors

% 080731 Version 0.2

%% Clear all varibles and close all graphs before the program starts
clear all;
close all;

%% Declare and initialise global constants. Do not change the values given
global STOP
global REVERSE
global LEFTFORWARD
global LEFTREVERSE
global RIGHTFORWARD
global RIGHTREVERSE
global FORWARD
global UGVID

%% Define the directions and UGVID
% The following values will be changed if you change the connections to
% the motors 
STOP = 0;
LEFTREVERSE = 4;
RIGHTREVERSE = 1;
LEFT = 6;
RIGHT = 9;
REVERSE = 5;
LEFTFORWARD = 2;
RIGHTFORWARD = 8;
FORWARD = 10;

%% Set your robot ID (00, 10 , 20, ...).
% Please use the UGVID assigned by tutors in week 2
UGVID = hex2dec('00');

%% Add your variables and constants here

%% Initialisation
% Initialise the car's motion, direction being one of the constants above
% and speed being an integer value between 0 (stopped) and 255 (full speed). 
direction = STOP;
speed1 = 100;       % Right Motor Speed
speed2 = 100;       % Left Motor Speed
servo = 128;
speed_diff = 0 ;                        % This value adjusts the motor speed so that the robot
speed1 = speed1 + round(speed_diff/2);  % can go straight. Obtain it by testing. Positive means
speed2 = speed2 + round(speed_diff/2);  % that robot needs to go more left.

%% Set up serial communication to the robot 
% You need to change the COM port to the USB XBee Explorer Board!!!
% Use X-CTU.exe program for Xbee to check the COM port for the Explorer board
theport = serial('COM7','BaudRate', 38400, 'Timeout', 0.1)  % Change the COM port
fclose(instrfind('Name', 'Serial-COM7'))                    % Close COM if already open
fopen(theport);
warning off MATLAB:serial:fread:unsuccessfulRead

t1 = clock();

%% Main While Loop
while 1
    %% Check the time taken to execute this loop - for debugging
    t2 = clock();
    cycletime = etime(t2, t1);
    t1 = clock();
    
    %% The following if loop ensure each cycle is more than 100ms,
    % and hence data has arrived from the robot
    if cycletime < 0.1
        pause(cycletime - 0.1);
    end

    %%   Enable these if you want to check your robot conditions
    %   direction = FORWARD;
    %   speed1 = 100 + round(speed_diff/2);  % Right Motor Speed
    %   speed2 = 100 - round(speed_diff/2);  % Left Motor Speed
    %   servo = 128;

    %% Limit the voltage (speed) to the motors (3v)
    % If you use NiMH batteries, the max voltage supply to the motor is 0.6*4.8 = 2.9v
    % If you use Alkaline batteries, the max voltage is 0.6*6 = 3.6v.
    % Since the internal resistance of Alkaline batteries is high, it is
    % ok to set the max voltage as 3.6v.
    % If you use other type of batteries, please change this value and make
    % sure the max voltage is less than 3v
    max_speeed = floor(0.6*256);    % Maximum speed/voltage

    speed1 = min(speed1,floor(0.6*256));
    speed2 = min(speed2,floor(0.6*256));
                                            
    %% Create the data packet and send it
    data_packet = [direction; speed1; speed2; servo];
    to_AVR(theport, UGVID+1, data_packet);
    
    %% Read serial data from the vehicle
    % If this happens too quickly (less than every 100ms), a complete packet
    % may not be received and the error 'whole packet isn't in the buffer' 
    % may be displayed.
    [fromAVR, numbytesread] = fread(theport);
    %fromAVR' % Display the received serial command
    
    %% The outputs of this function are the sensor data. 
    % IR is the distance to an object (in m) detected by the left IR sensor
    % US_range_mm is the distance (in mm) detected by the ultrasonic sensor.
    % You should consider filtering the ultrasonic sensor's output
    % Use these values to control your robot
    % If parse_status = 1, then the data has not been correctly parsed and
    % should be discarded. Otherwise, parse_status = 0
    % IR should be discarded for distances less than 5cm.
    % IR will return a value of 'inf' if nothing is detected
    [ID, direction, motorA, motorB, servo, US_range_mm, IR, parse_status] = parse_serial_robot(fromAVR', numbytesread);
    
    % Display sensor information, ID and cycletime
    fprintf(['US_range_mm:',num2str(US_range_mm),',\tIR:',num2str(IR),', \tID:',num2str(ID),', \tcycletime:',num2str(cycletime),'\n']);

    %% Sample code for controlling the robot
    % you may change anything below here
    
    %% Obstacle Avoidance using US
    % The following code does not use infrared range finder
    
    if(US_range_mm < 300) 
        direction = REVERSE;
        speed1 = 80;   % Right Motor Speed
        speed2 = 80;   % Left Motor Speed
    elseif(US_range_mm < 600)
        direction = LEFT;
        speed1 = 80;   % Right Motor Speed
        speed2 = 80;   % Left Motor Speed
    elseif(US_range_mm < 800)
        direction = FORWARD;
        speed1 = 100 + round(speed_diff/2); % Right Motor Speed
        speed2 = 100 - round(speed_diff/2);  % Left Motor Speed
    else
        direction = FORWARD;
        speed1 = 130 + round(speed_diff/2); % Right Motor Speed
        speed2 = 130 - round(speed_diff/2); % Left Motor Speed
    end
    
    %% Changing Servo position
    servo = servo + 8;
    if (servo > 255)
        servo = 1;
    end

end  % End of main while loop




