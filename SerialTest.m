% This is the serial Port testing function
disp('Available COM Ports Below: ');
AvailablePorts=serialportlist("available");
disp(AvailablePorts);
COMCheck=false;
while COMCheck==false
    COMPort=input('Type in the serial port (COM *number) that the Arduino is communicating through: \n','s');
    if max(strcmp(COMPort,AvailablePorts))
        COMCheck=true;
        Start=true;
        BAUDCheck=false;
        while BAUDCheck==false
            BAUD=str2double(input('Type in a BAUD Rate \n','s'));
            if isnan(BAUD)
                disp('Incorrect BAUD rate');
            else
                arduinoObj = serialport(COMPort,BAUD);
                configureTerminator(arduinoObj,"CR/LF");
                flush(arduinoObj);
                arduinoObj.UserData = struct("Data",[],"Count",1);
                BAUDCheck=true;
            end
        end
    else
        disp('Incorrect Port name');
    end
end

while Start
    data = readline(arduinoObj);
    disp(data);
end
