% collectPendData.m  -  MCHA3500 Lab 5, Problem 3
% Logs the potentiometer (A3 / PB0) through the STM32's ST-LINK COM port
% and plots it. Firmware side: Lab5.c, command "logPot [samples] [rate_Hz]".
%
% Based on the code given in the Lab 5 handout (3.1-3.2). Changes, written
% by Claude (Anthropic, AI): N and fs at the top are sent as logPot's R.1
% arguments (the defaults 400 and 200 give the same run as plain "logPot");
% flush() before the command; a clear error if a line times out; optional
% save. Close CoolTerm first: only one program can have the COM port open.

clear
clc

% remove any persistant serial connections
% (instrfind is gone in newer MATLAB releases, hence the try)
try
    if ~isempty(instrfind('Status', 'open'))
        fclose(instrfind);
    end
catch
end

% ---- Settings -----------------------------------------------------------
port     = "COM3";   % same port as CoolTerm uses
N        = 400;      % number of samples        (logPot default 400)
fs       = 200;      % sample rate, Hz          (logPot default 200)
saveName = "";       % e.g. "potData_2Hz.mat" to save, "" to not save

% Create connections to STM32 with 115200 baud rate and 0.5 second timeout
stm_device = serialport(port, 115200, 'Timeout', 0.5);
flush(stm_device);   % drop anything already waiting (e.g. Hello World!)

% Call the potentiometer data-logging code
writeline(stm_device, sprintf("logPot %d %g", N, fs));

% Initialise data structure for data
potData.time = zeros(N,1);
potData.voltage = zeros(N,1);
% Collect the expected number of samples
for i=1:N
    % Read in data from serial connection
    rxStr = readline(stm_device);
    if isempty(rxStr)
        clear stm_device
        error('No reply after %d samples. Is the board running Lab5.c and CoolTerm closed?', i-1);
    end
    % Separate string into parts
    separatedString = split(rxStr,',');
    % Convert data to numbers and store in data structure
    potData.time(i) = str2double(separatedString{1});
    potData.voltage(i) = str2double(separatedString{2});
end

% Plot results
plot(potData.time, potData.voltage, '+')
grid on
xlabel('Time')
ylabel('Voltage')

% Save if asked (be careful not to overwrite important data)
if strlength(saveName) > 0
    save(saveName, 'potData');
end

% Close serial connection and clean up
clear stm_device
