load('schoolzone.mat');
positiveInstances = schoolzone(:,1:2);
negativeFolder = 'C:\Users\Kashish Jain\Pictures\Camera Roll\school zone\Negatives';
trainCascadeObjectDetector('schoolzoneDetector.xml',positiveInstances, ...
    negativeFolder,'FalseAlarmRate',0.3,'NumCascadeStages',8);


detector = vision.CascadeObjectDetector('schoolzoneDetector.xml');

img = imread('test.jpg');

bbox = step(detector,img);

detectedImg = insertObjectAnnotation(img,'rectangle',bbox,'school zone');

figure; imshow(detectedImg);