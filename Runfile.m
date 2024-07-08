% Magnetic Levitation Ball System
% M.H.Ghadam / M.A.Mashayekhy
clc
clear
close all
%% Load and simulation MAGLEV system

load_system("Magnetic_Levitation_Part3.slx");
LBS = sim("Magnetic_Levitation_Part3");

% IPlant 
IPlant = LBS.IPlant.signals.values;
% OPlant 
OPlant = LBS.OPlant.signals.values;
% time
time = LBS.tout;


%% Make system
% these parameters are aproximated from 
% symulink model step response
A = -50;
B = 50;
C = 1;
D = 0;

Plant_Model = ss(A,B,C,D);

%% HW3
syms s 
fi = 1/(s+50);
% first input and initializing
out1_s = fi*(0.5/s)+fi*(0.5/s);
out1 = subs(ilaplace(out1_s),time);
n_out1 = double(out1);
% Secend input and initializing
out2_s = fi*(1/s)+fi*(1/s);
out2 = subs(ilaplace(out2_s),time);
n_out2 = double(out2);
%ANS
figure
plot(time,n_out1);
figure
plot(time,n_out2);
%% HW4
% Controlability
Controlability = ctrb(A , B);
CTR_RANK = rank(Controlability);
% Visibility
Visibility = obsv(A,C);
OBS_RANK = rank(Visibility);
%% Plot
figure
step(Plant_Model);
%% Neural Net fitting
% 
% x = IPlant';
% t = OPlant';
% 
% %Training Function
% trainFcn = 'trainbr'; % Bayesian regularization backpropagation
% 
% %Create a Fitting Network
% hiddenLayerSize = 35;
% 
% net = fitnet(hiddenLayerSize,trainFcn);
% 
% %Input and Output Pre/Post-Processing Functions
% 
% net.input.processFcns = {'removeconstantrows','mapminmax'};
% net.output.processFcns = {'removeconstantrows','mapminmax'};
% 
% %Setup Division of Data for Training, Validation, Testing
% 
% net.divideFcn = 'dividerand';  % Divide data randomly
% net.divideMode = 'sample';  % Divide up every sample
% net.divideParam.trainRatio = 65/100;
% net.divideParam.valRatio = 25/100;
% net.divideParam.testRatio = 10/100;
% 
% 
% 
% %Performance Function
% net.performFcn = 'sse';  % Sum squared error
% 
% %Plot Functions
% 
% net.plotFcns = {'plotperform','plottrainstate','ploterrhist', ...
%     'plotregression', 'plotfit'};
% 
% %Train the Network
% [net,tr] = train(net,x,t);
% 
% %Test the Network
% y = net(x);
% e = gsubtract(t,y);
% performance = perform(net,t,y);
% 
% %Recalculate Training, Validation and Test Performance
% trainTargets = t .* tr.trainMask{1};
% valTargets = t .* tr.valMask{1};
% testTargets = t .* tr.testMask{1};
% trainPerformance = perform(net,trainTargets,y);
% valPerformance = perform(net,valTargets,y);
% testPerformance = perform(net,testTargets,y);
% 
% 
% %View the Network
% view(net)
% 
% %Plots
% 
% figure, plotperform(tr)
% figure, plottrainstate(tr)
% figure, ploterrhist(e)
% figure, plotregression(t,y)
% figure, plotfit(net,x,t)
% 
% Deployment
% Change the (false) values to (true) to enable the following code blocks.
% See the help for each generation function for more information.
% if (false)
%     %Generate MATLAB function for neural network for application
%     %deployment in MATLAB scripts or with MATLAB Compiler and Builder
%     %tools, or simply to examine the calculations your trained neural
%     %network performs.
%     genFunction(net,'myNeuralNetworkFunction');
%     y = myNeuralNetworkFunction(x);
% end
% if (false)
%     %Generate a matrix-only MATLAB function for neural network code
%     %generation with MATLAB Coder tools.
%     genFunction(net,'myNeuralNetworkFunction','MatrixOnly','yes');
%     y = myNeuralNetworkFunction(x);
% end
% if (false)
%     %Generate a Simulink diagram for simulation or deployment with.
%     %Simulink Coder tools.
%     gensim(net);
% end


