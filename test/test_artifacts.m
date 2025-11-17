%% process throught matlab
% not modification needed for these informations
datapath = '/home/paolo/cvsa/ic_cvsa_ws/src/artifacts_cvsa/test/';
filein = [datapath ,'rawdata.csv'];
data = readmatrix(filein);
filterOrder = 4;
band = [8, 14];
bufferSize = 512;
sampleRate = 512;
frameSize = 32;
nsamples = size(data, 1);
nchannels = size(data, 2);

%% apply the processing
disp(['      [INFO] start processing like ros for band ' num2str(band(1)) '-' num2str(band(2))]);
header.Label = [{'FP1'}    {'FP2'}    {'F3'}    {'FZ'}    {'F4'}    {'FC1'}    {'FC2'}    {'C3'}    {'CZ'}    {'C4'}    {'CP1'}    {'CP2'}  ...
    {'P3'}    {'PZ'}    {'P4'}    {'POZ'}    {'O1'}    {'O2'}    {'EOG'}    {'F1'}    {'F2'} ...
    {'FC3'}    {'FCZ'}    {'FC4'}    {'C1'}    {'C2'}    {'CP3'}    {'CP4'}    {'P5'}    {'P1'}    {'P2'}    {'P6'}    ...
    {'PO5'}    {'PO3'}    {'PO4'}    {'PO6'}    {'PO7'}    {'PO8'}    {'OZ'}];
header.SampleRate = 512;
chunkSize = 32;
eog.filterOrder = 4;
eog.band = [1 10];
eog.label = {'FP1', 'FP2', 'EOG'};
eog.h_threshold = 70;
eog.v_threshold = 70;
muscle.filterOrder = 4;
muscle.freq = 2; % remove antneuro problems
muscle.threshold = 160;
artifact = artifact_rejection(data, header, nchannels, bufferSize, chunkSize, eog, muscle);

%% Load file of rosneuro
SampleRate = 16;
start = 1;

files{1} = [datapath 'node/artifacts.csv'];

for i=1:length(files)
    file = files{i};
    disp(['Loading file: ' file])
    ros_data = readmatrix(file);
    matlab_data = artifact;
    c_title = "processed with ros node simulation";
    nsamples = size(matlab_data,1);
%     t = 0:1/SampleRate:nsamples/SampleRate - 1/SampleRate;
    t = 0:nsamples-1;


    figure;
    subplot(2, 1, 1);
    hold on;
    plot(t(start:end), ros_data(start:size(t,2)), 'b', 'LineWidth', 1);
    plot(t(start:end), matlab_data(start:size(t, 2)), 'r');
    legend('rosneuro', 'matlab');
    hold off;
    grid on;

    subplot(2,1,2)
    bar(t(start:end), abs(ros_data(start:size(t,2))- matlab_data(start:size(t,2))));
    grid on;
    xlabel('time [s]');
    ylabel('amplitude [uV]');
    title('Difference')

    sgtitle(['Evaluation' c_title])
end