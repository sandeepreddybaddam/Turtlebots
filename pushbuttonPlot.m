function pushbuttonPlot()
    f = figure;
    ax = axes(f);
    ax.Units = 'pixels';
    ax.Position = [75 75 325 280];
    c = uicontrol;
    c.String = 'Simulate';
    c.Callback = @plotButtonPushed;

    uicontrol('style','text','string','Stability Condition: Kp * Kd * Vx >= Ki','Fontsize', 10, 'FontWeight', 'Bold', 'Position',[40 380 280 20]);
    c1 = uicontrol(f, 'Style','edit', 'Position', [450 380 60 20]);
%     uicontrol('style','text','string','Kp','units','centimeters','position',[1 1 1 0.5]);
    uicontrol('style','text','string','Kp','Fontsize', 10, 'Position',[420 380 24 20]);
    uicontrol(c1);

    c2 = uicontrol(f, 'Style','edit', 'Position', [450 350 60 20]);
    uicontrol('style','text','string','Kd','Fontsize', 10, 'Position',[420 350 24 20]);
    uicontrol(c2);
    c3 = uicontrol(f, 'Style','edit', 'Position', [450 320 60 20]);
    uicontrol('style','text','string','Ki', 'Fontsize', 10, 'Position',[420 320 24 20]);
    uicontrol(c3);
    c4 = uicontrol(f, 'Style','edit', 'Position', [450 290 60 20]);
    uicontrol('style','text','string','Vx', 'Fontsize', 10, 'Position',[420 290 24 20]);
    uicontrol('style','text','string','m/s', 'Fontsize', 10, 'Position',[510 290 30 20]);
    uicontrol(c4);

    uicontrol('style','text','string','UPC Laboratory, University of Washington', 'FontAngle', 'italic', 'Position',[300 10 300 20]);

    function plotButtonPushed(src, event)
        s = tf('s');        
        Kp = str2num(c1.String);
        Kd = str2num(c2.String);
        Ki = str2num(c3.String);
        Vx = str2num(c4.String);
        G = ((Kp*Vx*s)+(Kd*Vx*s*s)+(Ki*Vx))/((s^3)+(Kd*Vx*s^2)+(Kp*Vx*s)+(Ki*Vx));
        [y,t] = step(G);
        plot(t,y)
        title('System response')
        xlabel('Time(sec)')
        ylabel('Amplitude')
        grid on
        S = stepinfo(G);
        Ts = num2str(S.SettlingTime);
        OS = num2str(S.Overshoot);  %# Get the value in the Overshoot field
        peakTime = num2str(S.PeakTime);
        uicontrol('style','text','string','Transient Parameters', 'Fontsize', 10, 'FontWeight', 'Bold', 'Position',[410 200 150 20]);
        string = append('Settling Time(sec): ', Ts, sprintf('\n'), 'Overshoot(%): ', OS, sprintf('\n'), 'Peaktime(sec): ', peakTime);
        uicontrol('style','text','string',string, 'Position',[410 100 150 100]);
    end
end

