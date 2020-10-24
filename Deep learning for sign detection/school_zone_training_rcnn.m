% layers = [ imageInputLayer([28 28 1]) 
%     convolution2dLayer(3,8,'Padding','same') 
%     batchNormalizationLayer
%     reluLayer
%     maxPooling2dLayer(2,'Stride',2) 
%     
%     convolution2dLayer(3,16,'Padding','same') 
%     batchNormalizationLayer
%     reluLayer
%     maxPooling2dLayer(2,'Stride',2) 
%     
%     convolution2dLayer(3,32,'Padding','same') 
%     batchNormalizationLayer
%     reluLayer
%     
%     fullyConnectedLayer(10) 
%     softmaxLayer
%     classificationLayer];

trainingData = objectDetectorTrainingData(gTruth1);
options = trainingOptions('sgdm', ...
    'MiniBatchSize', 128, ...
    'InitialLearnRate', 1e-3, ...
    'LearnRateSchedule', 'piecewise', ...
    'LearnRateDropFactor', 0.1, ...
    'LearnRateDropPeriod', 100, ...
    'MaxEpochs', 100, ...
    'Verbose', true,...
    'ExecutionEnvironment','cpu');


sz_rcnn = trainRCNNObjectDetector(trainingData, cifar10Net, options, ...
    'NegativeOverlapRange', [0 0.3], 'PositiveOverlapRange',[0.5 1])