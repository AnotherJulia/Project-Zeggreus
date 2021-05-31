% Create a biquad antenna backed with reflector
% Generated by MATLAB(R) 9.9 and Antenna Toolbox 4.3.
% Generated on: 23-May-2021 03:59:11

%% Antenna Properties 
% Design antenna at frequency 2450000000Hz
antennaObject = design(biquad,2450000000);
% Define backing structure 
antennaObject = reflector('Exciter', antennaObject);
% Adjust backing structure dimensions
antennaObject.GroundPlaneLength = 0.123;
antennaObject.GroundPlaneWidth = 0.095;
antennaObject.Spacing = 0.0151;
antennaObject.EnableProbeFeed = 0;
antennaObject.Tilt = 0;
antennaObject.TiltAxis = [1 0 0];
% Properties changed 
antennaObject.Exciter.ArmLength = 0.03;
antennaObject.Exciter.Width = 0.00178;
antennaObject.Exciter.Tilt = 90;
% Update substrate properties 
antennaObject.Substrate.Name = 'Air';
antennaObject.Substrate.EpsilonR = 1;
antennaObject.Substrate.LossTangent = 0;
antennaObject.Substrate.Thickness = 0.0151;

%% Antenna Analysis 
% Define plot frequency 
plotFrequency = 2450000000;
% Define frequency range 
freqRange = (2205:24.5:2695) * 1e6;
% show for reflector
figure;
show(antennaObject) 
% impedance for reflector
figure;
impedance(antennaObject, freqRange) 
% s11 for reflector
figure;
s = sparameters(antennaObject, freqRange); 
rfplot(s) 
% pattern for reflector
figure;
pattern(antennaObject, plotFrequency) 
