x1 = 3.5*ecg(2700).'; 
y1 = sgolayfilt(kron(ones(1,13),x1),0,21);
n = 1:30000;
del = round(2700*rand(1));
signal= y1(n + del);
t = 0.00025:0.00025:7.5;
%subplot(3,2,1);
figure ;
plot(t,signal);
axis([0 2.5 -5 5]);
grid;
xlabel('Time [sec]');
ylabel('Voltage [mV]');
title('a) Heartbeat Signal');
nvar  = 0.5;                  % Noise variance
%noise = randn(size(signal))*nvar;
noise=3*sin(2*pi*50*t);
%subplot(3,2,2)
figure 
plot(t,noise);
axis([0 2.5 -4 4]);
title('b) Noisy time domain signal')


nfilt  = fir1(31,0.5);             % 31st order Low pass FIR filter
fnoise = filter(nfilt,1,noise);    % Filtering the noise
d  = signal+fnoise;
%subplot(3,2,3)
figure 
plot(t,d);
axis([0 2.5 -5 5]);
title('c) Signal + Noise')


D = fft(d,512);
Pyy = D.* conj(D) / 512;
f = 1000*(0:256)/512;
%subplot(3,2,4);
figure 
plot(f(1:50),Pyy(1:50))
title('d) Power spectral density')
xlabel('Frequency (Hz)')


M = 32;                    % Filter order
lam = 1;                   % Exponential weighting factor
delta = 0.1;               % Initial input covariance estimate
w0 = zeros(M,1);           % Initial tap weight vector
P0 = (1/delta)*eye(M,M);   % Initial setting for the P matrix
Zi = zeros(M-1,1);         % FIR filter initial states

% Running the RLS adaptive filter for 1000 iterations.  The plot shows the
% convergence of the adaptive filter response to the response of the FIR filter.
Hadapt = adaptfilt.rls(M,lam,P0,w0,Zi);
Hadapt.ResetBeforeFiltering = 'off';
[y,e] = filter(Hadapt,noise,d);
H = abs(freqz(Hadapt,1,64));
H1 = abs(freqz(nfilt,1,64));

%subplot(3,2,5);
figure
plot(t,signal,'* Y',t,e); grid;
title('e) Original information bearing signal and the error signal');
legend('Original Signal','Error Signal');
axis([0 2.5 -5 5]);

E = fft(e,512);
PYY = E.* conj(E) / 512;
F = 1000*(0:256)/512;
figure 
%subplot(3,2,6);
plot(F(1:50),PYY(1:50))
title('f) Power spectral density')
xlabel('Frequency (Hz)')

