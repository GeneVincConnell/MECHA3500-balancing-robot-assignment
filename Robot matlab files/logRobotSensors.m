function T = logRobotSensors(port, duration_s, outFile, plotIt, opts)
%LOGROBOTSENSORS  Record the balancing robot's encoders and IMU, save as CSV.
%
%   Timed run (firmware command logAll): motor PWM off, gyro calibrated,
%   then duration_s of samples at 500 Hz
%     T = logRobotSensors                    finds the ST-LINK COM port, 3 s,
%                                            saves sensorLog_<date_time>.csv
%     T = logRobotSensors("COM5")            pick the port yourself
%     T = logRobotSensors("COM5", 2, "run1.csv", false)   2 s, no plot
%
%   Fall test (firmware command logFall): both motors hold the wheels still
%   (omega = 0), you let go of the robot, and the fall is saved from PreS
%   before to PostS after the moment the tilt has moved TriggerDeg from
%   where you held it. That moment is time 0 (earlier times are negative).
%     T = logRobotSensors("COM5", Fall=true)          1 s, 2 s, 3 deg;
%                                                     saves fallLog_<date_time>.csv
%     T = logRobotSensors("COM5", Fall=true, PreS=0.5, PostS=1.5, TriggerDeg=2)
%     T = logRobotSensors("COM5", 3, "fall1.csv", true, Fall=true)
%   (duration_s is not used for a fall test). What happens:
%     1. the robot stops the STEP PWM and switches both drivers on to hold
%        (BATTERY ON) at the run current. HOLD THE ROBOT UPRIGHT: the rotors
%        snap to their nearest step position as the drivers switch on;
%     2. it recalibrates the gyro (~1 s): keep it upright and STILL;
%     3. it prints "Armed": let go without pushing, within 30 s, on a floor
%        the wheels cannot slip on;
%     4. it records PostS after the trigger, puts the motors back as they
%        were, and sends the samples (~6 ms per sample at 115200 baud).
%
%   Columns (raw, as the robot sent them)
%     time_s                 s since the first sample (logAll) or since the
%                            trigger (logFall); real sample times
%     enc_L_counts, enc_R_counts   counts since the first sample, 4000/rev,
%                            + = forwards on both wheels
%     acc_x/y/z_raw          acc_lsb_per_g (16384) per g
%     gyro_x/y/z_raw         gyro_lsb_per_dps (32.8 at +/-1000 deg/s) per deg/s,
%                            bias NOT removed
%     theta_rad, theta_dot_rad_s   the firmware's tilt estimate, + = forwards
%   Columns added here (SI)
%     wheel_L_rad, wheel_R_rad     motor angle phi (rad), relative to the chassis
%     acc_x/y/z_m_s2, gyro_x/y/z_rad_s
%     theta_dot_gyro_rad_s   tilt rate from the gyro, bias removed, + = forwards
%     theta_gyro_rad         its integral (trapezoidal), starting from the
%                            firmware's theta_rad at the first sample.
%                            Use this one to curve-fit a fall: theta_rad (the
%                            complementary filter) is pulled towards the
%                            accelerometer angle, which is wrong while the
%                            body accelerates.
%   The MPU-6050's own 42 Hz filter delays the IMU columns by ~5 ms. That only
%   shifts the time axis; it does not change the shape of the fall.
%
%   T.Properties.UserData holds the numbers from the #INFO and #CSV_END lines
%   (gyro scale and bias, sample rate, longest gap, ...). They are also saved
%   next to the CSV as <name>_info.txt. Load the samples again any time with
%       T = readtable("fallLog_....csv");
%
%   Close CoolTerm (or anything else using the COM port) first.
%
%   PROVENANCE: AI-generated. Written by Claude (Anthropic), 14 Nov 2026,
%   for SensorLog.c in the MCHA3500 balancing-robot firmware.
%   Updated by Claude, 14 Nov 2026: fall test (Fall=true), 500 Hz, gyro scale
%   read from the robot, theta_gyro_rad and theta_dot_gyro_rad_s columns.

arguments
    port       (1,1) string = ""
    duration_s (1,1) double {mustBeInRange(duration_s, 0.1, 4.5)} = 3
    outFile    (1,1) string = ""
    plotIt     (1,1) logical = true
    opts.Fall       (1,1) logical = false
    opts.PreS       (1,1) double {mustBeInRange(opts.PreS, 0, 4.4)} = 1
    opts.PostS      (1,1) double {mustBeInRange(opts.PostS, 0.1, 4.5)} = 2
    opts.TriggerDeg (1,1) double {mustBeInRange(opts.TriggerDeg, 0.5, 20)} = 3
end

BAUD           = 115200;
COUNTS_PER_REV = 4000;      % Encoder.h: 1000 PPR x 4
GYRO_SIGN_X    = -1;        % Robot.h: theta_dot = -gyro x
G              = 9.80665;   % m/s^2
MAX_S          = 4.5;       % SensorLog.h: SENSORLOG_MAX_S
FALL_TIMEOUT_S = 30;        % SensorLog.h: SENSORLOG_FALL_TIMEOUT_S

%% What to ask the robot for
if opts.Fall
    if opts.PreS + opts.PostS > MAX_S
        error("logRobotSensors:fallLength", ...
            "PreS + PostS must be at most %g s (the robot's buffer).", MAX_S);
    end
    command = sprintf("logFall %g %g %g", opts.PreS, opts.PostS, opts.TriggerDeg);
    waitS   = 0.3 + 1.5 + FALL_TIMEOUT_S + opts.PostS + 10;   % hold, calibrate, wait, record
    prefix  = "fallLog_";
else
    command = sprintf("logAll %g", duration_s);
    waitS   = 1.5 + duration_s + 10;                          % calibrate, record
    prefix  = "sensorLog_";
end
if outFile == ""
    outFile = prefix + string(datetime("now", "Format", "yyyyMMdd_HHmmss")) + ".csv";
end

%% Open the port
if port == ""
    ports = serialportlist("available");
    if numel(ports) ~= 1
        error("logRobotSensors:port", ...
            "Found %d free serial ports (%s). Call logRobotSensors(""COMx"") with the " + ...
            "STLink Virtual COM Port from Device Manager.", numel(ports), strjoin(ports, ", "));
    end
    port = ports;
end
s = serialport(port, BAUD, "Timeout", waitS);
configureTerminator(s, "LF");
closePort = onCleanup(@() delete(s));
flush(s);                                                 % drop anything old

%% Start a run
writeline(s, command);
if opts.Fall
    fprintf("%s sent on %s. Hold the robot upright and still until it says Armed, then let go.\n", ...
        command, port);
else
    fprintf("%s sent on %s. Keep the robot still until it says Recording.\n", command, port);
end

% Status lines and #INFO until #CSV_BEGIN (or #ERROR)
info      = struct();
infoLines = strings(0, 1);
while true
    line = readOneLine(s, "the robot to finish recording");
    if startsWith(line, "#CSV_BEGIN")
        break
    elseif startsWith(line, "#ERROR")
        error("logRobotSensors:robot", "Robot: %s", strtrim(extractAfter(line, "#ERROR")));
    elseif startsWith(line, "#INFO")
        info = parseKeyValues(line, info);
        infoLines(end + 1) = line; %#ok<AGROW>
    elseif line ~= ""
        fprintf("  robot: %s\n", line);
    end
end

%% Receive the CSV
s.Timeout = 5;                                            % lines now arrive back to back
header = split(readOneLine(s, "the CSV header"), ",").';
rows   = strings(2500, 1);
n      = 0;
fprintf("  receiving");
while true
    line = readOneLine(s, "the CSV rows");
    if startsWith(line, "#CSV_END")
        footer = line;
        break
    end
    n = n + 1;
    rows(n) = line;
    if mod(n, 200) == 0
        fprintf(".");
    end
end
fprintf(" %d rows\n", n);
if n == 0
    error("logRobotSensors:empty", "The robot sent no samples (%s).", footer);
end
info = parseKeyValues(footer, info);
infoLines(end + 1) = footer;

%% Table, SI columns, save
parts = split(rows(1:n), ",");
if n == 1
    parts = parts.';
end
data = str2double(parts);
if size(data, 2) ~= numel(header) || any(isnan(data), "all")
    error("logRobotSensors:parse", "Could not read every row as %d numbers.", numel(header));
end
T = array2table(data, "VariableNames", header);

% Scale factors from the robot. Firmware from before 14 Nov 2026 sent no
% #INFO line and ran the gyro at +/-250 deg/s (131 per deg/s).
gyroLsb = getField(info, "gyro_lsb_per_dps", 131);
accLsb  = getField(info, "acc_lsb_per_g", 16384);
biasDps = getField(info, "gyro_bias_x_dps", 0);

T.wheel_L_rad  = T.enc_L_counts * 2*pi / COUNTS_PER_REV;
T.wheel_R_rad  = T.enc_R_counts * 2*pi / COUNTS_PER_REV;
T.acc_x_m_s2   = T.acc_x_raw  / accLsb * G;
T.acc_y_m_s2   = T.acc_y_raw  / accLsb * G;
T.acc_z_m_s2   = T.acc_z_raw  / accLsb * G;
T.gyro_x_rad_s = T.gyro_x_raw / gyroLsb * pi/180;
T.gyro_y_rad_s = T.gyro_y_raw / gyroLsb * pi/180;
T.gyro_z_rad_s = T.gyro_z_raw / gyroLsb * pi/180;
T.theta_dot_gyro_rad_s = GYRO_SIGN_X * (T.gyro_x_raw / gyroLsb - biasDps) * pi/180;
T.theta_gyro_rad       = T.theta_rad(1) + cumtrapz(T.time_s, T.theta_dot_gyro_rad_s);
T.Properties.UserData  = info;

writetable(T, outFile);
[folder, name] = fileparts(outFile);
infoFile = fullfile(folder, name + "_info.txt");
fid = fopen(infoFile, "w");
if fid > 0
    fprintf(fid, "%s\n", infoLines);
    fclose(fid);
end
span = T.time_s(end) - T.time_s(1);
fprintf("Saved %d samples, %.3f s at %.1f Hz, to %s (+ %s)\n", n, span, ...
    (n - 1) / max(span, eps), outFile, infoFile);

%% Warnings
if getField(info, "buffer_full", 0)
    warning("logRobotSensors:bufferFull", "The robot's buffer filled up: the run stopped early.");
end
if getField(info, "pwm_restarted", 0) > 0
    warning("logRobotSensors:pwm", ...
        "Something in the firmware turned the motor PWM back on %d time(s); SensorLog stopped it again.", ...
        info.pwm_restarted);
end
if getField(info, "i2c_errors", 0) > 0
    warning("logRobotSensors:i2c", ...
        "%d MPU-6050 read(s) failed; the robot repeated the previous values there.", info.i2c_errors);
end
period_ms = getField(info, "sample_period_us", 2000) / 1000;
if getField(info, "max_gap_ms", 0) > 1.5 * period_ms
    warning("logRobotSensors:gap", ...
        "The longest gap between samples was %.1f ms (expected %.1f ms).", info.max_gap_ms, period_ms);
end
if any(abs(T.gyro_x_raw) >= 32767)
    warning("logRobotSensors:gyroClipped", ...
        "gyro_x reached the end of its range (%.0f deg/s) at t = %.3f s; theta_gyro_rad is wrong after that.", ...
        32767 / gyroLsb, T.time_s(find(abs(T.gyro_x_raw) >= 32767, 1)));
end

%% Quick look
if plotIt
    figure("Name", outFile);
    tiledlayout(3, 1, "TileSpacing", "compact");
    ax = gobjects(3, 1);
    ax(1) = nexttile; plot(T.time_s, [T.theta_gyro_rad, T.theta_rad] * 180/pi); grid on
    ylabel("\theta (deg)"); legend("gyro integrated", "firmware", "Location", "best");
    ax(2) = nexttile; plot(T.time_s, [T.theta_dot_gyro_rad_s, T.theta_dot_rad_s]); grid on
    ylabel("d\theta/dt (rad/s)"); legend("gyro (MATLAB)", "firmware", "Location", "best");
    ax(3) = nexttile; plot(T.time_s, [T.wheel_L_rad, T.wheel_R_rad]); grid on
    ylabel("\phi, motor angle (rad)"); legend("left", "right", "Location", "best");
    xlabel("time (s)");
    linkaxes(ax, "x");
    if opts.Fall
        for k = 1:numel(ax)
            xline(ax(k), 0, "--", "trigger", "HandleVisibility", "off");
        end
    end
end
end

function info = parseKeyValues(line, info)
% "#INFO a=1 b=2.5 ..." / "#CSV_END ..." -> info.a = 1, info.b = 2.5, ...
kv = regexp(line, "(\w+)=(-?[\d.]+)", "tokens");
for k = 1:numel(kv)
    info.(kv{k}(1)) = str2double(kv{k}(2));
end
end

function v = getField(info, name, default)
if isfield(info, name) && ~isnan(info.(name))
    v = info.(name);
else
    v = default;
end
end

function line = readOneLine(s, waitingFor)
% readline() returns [] with a warning on timeout; make that an error instead
w = warning("off", "transportlib:client:ReadlineWarning");
restoreWarning = onCleanup(@() warning(w));
line = readline(s);
if isempty(line)
    error("logRobotSensors:timeout", ...
        "Timed out waiting for %s. Is SensorLog in the firmware, and is SensorLog_Update() " + ...
        "called after IMU_Update() in main.c? For a fall test, did the robot fall within 30 s?", ...
        waitingFor);
end
line = strtrim(string(line));
end
