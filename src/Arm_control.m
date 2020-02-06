
%fclose(instrfind);

ard= serial('COM3','BaudRate',57600);

Fs=256; % set on ard code

Update_motor = 0.03; %in seconds

Twindow =1; % number of seconds to have on screen at once
plotsize=Twindow*Fs;
chn_num=1;

time=(0:plotsize-1)/Fs;

data = zeros(chn_num,plotsize);
rms  = zeros(chn_num,plotsize);
motor= zeros(chn_num,plotsize);

TXBuf = zeros(10,1);

packetsize=17;
numread=20; %max 30 as 512 bytes inbuffer
num_ave = 40;


[bh,ah] = butter(3,3/(Fs/2),'high');
[bl,al] = butter(3,30/(Fs/2),'low');
%% Graph Stuff

subplot(3,1,1);
plotGraph1 = plot(time,data(1,:),'-',...
    'LineWidth',2,...
    'MarkerFaceColor','w',...
    'MarkerSize',2);

title('EMG','FontSize',20);
xlabel('Time, seconds','FontSize',15);
ylabel('Voltage, V','FontSize',15);

ylim([0,1000]);
xlim([0,1]);

subplot(3,1,2);
plotGraph2 = plot(time,rms(1,:),'-',...
    'LineWidth',2,...
    'MarkerFaceColor','w',...
    'MarkerSize',2);

title('RMS EMG','FontSize',20);
xlabel('Time, seconds','FontSize',15);
ylabel('RMS Voltage, V','FontSize',15);

ylim([0,100]);
xlim([0,1]);

subplot(3,1,3);
plotGraph3 = plot(time,motor(1,:),'-',...
    'LineWidth',2,...
    'MarkerFaceColor','w',...
    'MarkerSize',2);

title('Motor command','FontSize',20);
xlabel('Time, seconds','FontSize',15);
ylabel('RMS Voltage, V','FontSize',15);

ylim([0,180]);
xlim([0,1]);

drawnow
%% Reading data setup
fopen(ard);
pause(1);
iSample =1;
wSamp=1;
fwrite(ard,'S');
                      
   %%               
M0 = 120;
step = 0.4;
% 
while ishandle(plotGraph1) %&& ishandle(plotGraph2)
    
    if ard.BytesAvailable >= numread*packetsize
        
        for iRead = 1:numread
            
            [A,count] = fread(ard,packetsize,'uint8');
               
            data(1,iSample)=double(swapbytes(typecast(uint8(A(5:6)), 'uint16')));
           % data(2,iSample)=double(swapbytes(typecast(uint8(A(7:8)), 'uint16')));
            
            if iSample > num_ave
                rms(1,iSample) = std(filtfilt(bh,ah,data(1,iSample-num_ave:iSample)));
                rms(1,iSample) = mean(rms(1,iSample-5:iSample));
            else
                rms(1,iSample) = std(filtfilt(bh,ah,data(1,[end-num_ave+iSample:end,1:iSample])));
                rms(1,iSample) = mean(rms(1,[end-5+iSample:end,1:iSample]));
            end
            
            P = rms(1,iSample);
%% THRESHOLD CONDITION controls the speed          
            if P<70
                M0=M0+step;
            elseif P<90
                M0=M0-step;
            elseif P<100
                M0=M0-2*step;
            else
                M0=M0-3*step;
            end
            
            
            
            
%%            
            if M0>170
                M0=170;
            elseif M0<10
                M0=20;
            end
            
            motor(1,iSample)=M0;
 
            iSample = iSample +1;
            wSamp = wSamp+1;
            if iSample > plotsize
                iSample =1;
                
            end
            
        end
        
        try
            set(plotGraph1,'YData',data(1,:));
            set(plotGraph2,'YData',rms(1,:));
            set(plotGraph3,'YData',motor(1,:));
            drawnow;
        
        catch
   end
        
   packetsleft=floor(ard.BytesAvailable/packetsize);
   if packetsleft > 20 %max is 30 but lets leave some room
       fprintf(2,'Update rate is too slow!: %d \n',packetsleft);
   end
   
   
   

   
    end
 %% 
  if wSamp>plotsize*Update_motor
      
     M = uint16([M0,M0,M0,M0,180-M0]);
     for CurrCh = 1:5
       P = typecast(uint16(M(CurrCh)),'uint8');
       TXBuf(2*CurrCh-1)=P(2);
       TXBuf(2*CurrCh)=P(1);    
     end

    for i = 1:10
      fwrite(ard,TXBuf(i));
    end
      wSamp=1;
  end
    
end

fclose(ard);








