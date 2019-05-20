close all
%{
    brief this code is used to analyze the sound of a string produced
    by a guitar. Then, using information obtained, emulate this signal
    in a embedded system
%}

%Loading the audio archives
[string,Fs] = audioread('string5a.wav'); %Lee archivo de audio
%Why audio signal string has two dimmensions ?
%Using only one, the two dimmensions are the same
string = string(:,1);
string_sound = audioplayer(string,Fs);
%play(string_sound)

%Plot audio of string
%Creating vector of time and normalizing it to lenght of audio (5 s)    
time = 0 : 1 : size(string,1) - 1;
time = ( time / size(string,1) ) .* 5;
time = time';

figure;
plot( time, string );
title(' Sonido de cuerda de guitarra ');
xlabel('Tiempo (s)')
ylabel('Amplitud')

% Sampling string sound to know its behavior
string_sampled = 1:1:400;

for i= 1: 1: 400
    string_sampled(i) = string(i .* round( size(string,1)/400 ));
    
end

%figure;
%plot( string_sampled );
brush on
title('Se�al de audio: 400 mueplastras ');
ylabel('Amplitud');
xlabel('muestra')



% Computing frequency spectrum of audio signal (string)
%Graficando el espectro de frecuencias de la se�al de audio cargada
% anteriormente.
W_Hertz = 0:1 : size(string,1)- 1;
W_Hertz = (Fs / size(string,1)) .* W_Hertz;
% Computing frequency spectrum of sound
freq_spctrm = fft(string(:,1),size(W_Hertz,2) );

figure;

plot( W_Hertz, abs( freq_spctrm / max(freq_spctrm)))
title(' Espectro de frecuencias de sting5a.wav ')
xlabel(' Frecuencia (Hz)')
figure;
%spectrogram(string,100,'yaxis')        
note = 1:1:238464 ;
note(:) = 0;

f = 89;
           %   110        220       330        440    550    660      770
           %   880        990 1210
           %     
amplitudes = [ 0.9056       1      0.2456     0.0325   0     0.1215  0.1075     0.1017    0.07540  0  0.02388  0.01797  0  0.006781  0  0.01167   0.04141  0.0361 0.02958 0.009368 0.01406 0.01873      ];
for i=1:size(amplitudes,2)
   temp =  amplitudes(i).*sin(2.*pi*time.*f.*i);
   note(:) = note(:) + temp;
  
   
end


plot(time,note)



%generating a envolvent function (like a window)
figure;
%normalizing audio sampled when divinding its abs into peak value;
attack = 0:1:7152;
attack = attack(:).*139.821e-06 ;

decay = 7153: 1 : 114462;
decay = decay(:).*-8.38699e-06 + 1.0599;


sustain = 114463:1:212232;
sustain(:) = 0.0907;

release = 212233: 1 : 238464;
release = release(:).*-3.81228e-06 + 909.092e-3;
release = release .* 0.9181;
envelope =  1:1: size(string,1);
envelope(1:7153) = 0;
envelope(7154:114463) = exp(-time(7154:114463));
envelope(114464:212233) = sustain;
envelope(212233:238464)= release;


note_enveloped = note.*envelope;

s = audioplayer(note_enveloped,Fs);
play(s);

plot(note_enveloped)

% Sampling string sound to know its behavior
string_sampled = 1:1:8000;
string_sampled(:) = 0;
for i= 1: 7948
    string_sampled(i) = note_enveloped(i .* round( size(note_enveloped,2)/8000 ));
end

string_sampled = string_sampled(:)+ 1.5415;
to_dac = uint16(4096 * mat2gray(string_sampled));
 fprintf(' P1  '); 
for i=1:1000
   fprintf(' %i,',to_dac(i)) 
   
end
fprintf(' \n'); 
fprintf('P2   ');
for i=2001:3000
   fprintf(' %i,',to_dac(i)) 
   
end
fprintf(' \n'); 
fprintf('P3  ,');
for i=3001:4000
   fprintf(' %i,',to_dac(i)) 
   
end
fprintf(' \n,'); 

fprintf('P4  ,');
for i=4001:5000
   fprintf(' %i,',to_dac(i)) 
   
end
fprintf(' \n,'); 

fprintf('P5  ,');
for i=5001:6000
   fprintf(' %i,',to_dac(i)) 
   
end
fprintf(' \n,'); 

fprintf('P6  ,');
for i=6001:7000
   fprintf(' %i,',to_dac(i)) 
   
end
fprintf(' \n,'); 
 
fprintf('P6  ,');
for i=7001:8000
   fprintf(' %i,',to_dac(i)) 
   
end
fprintf(' \n,'); 
