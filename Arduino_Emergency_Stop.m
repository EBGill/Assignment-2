function [outputArg1,outputArg2] = Arduino(inputArg1,inputArg2)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% Initialize the Arduino board

a = arduino('COM3'); % Change 'COM3' to your Arduino's COM port

% Define pins for input and output
EmergStopButton = 'D2';    % Digital input pin
ResetSwitch = 'D3';        % Digital input pin
ReactivateButton = 'D4';   % Digital input pin
EmergStopStatus = 'D8';     % Digital output pin
LED1 = 'D9';               % Digital output pin
LED2 = 'D10';              % Digital output pin

% Set initial system state
systemState = 0;  % 0 for stopped, 1 for waiting, 2 for active

% Main loop to continuously read and control the system
while true
    % Read the state of buttons
    EmergStopButtonState = readDigitalPin(a, EmergStopButton);
    ResetSwitchState = readDigitalPin(a, ResetSwitch);
    ReactivateButtonState = readDigitalPin(a, ReactivateButton);
    
    % Emergency Stop pressed
    if EmergStopButtonState == 1 && systemState == 2
        writeDigitalPin(a, EmergStopStatus, 1);
        writeDigitalPin(a, LED1, 1);
        writeDigitalPin(a, LED2, 1);
        systemState = 0;
        disp('EMERGENCY STOP ON');
    end
    
    % Wait for Reactivate Button press
    if systemState == 0 && ReactivateButtonState == 1
        systemState = 1;
        writeDigitalPin(a, LED1, 1);
        disp('Pressed Reactivate Button, Toggle Switch to Resume Full Movments');
        pause(0.1);  % Debounce
    end
    
    % Reactivate the system with the switch
    if systemState == 1 && ReactivateButtonState == 0 && ResetSwitchState == 1
        pause(0.1);  % Debounce
        if ResetSwitchState == 1
            writeDigitalPin(a, EmergStopStatus, 0);
            writeDigitalPin(a, LED1, 0);
            writeDigitalPin(a, LED2, 1);
            systemState = 2;
            disp('System Reactivated and Fully Functioning');
        end
    end
end

% Close the Arduino connection when done (this part will never be reached)
clear a;

end