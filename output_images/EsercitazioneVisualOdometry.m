% Esercitazione Visual Odometry
% Corso di laurea magistrale - Ingegneria Aerospaziale -
% A.A. 2023-2024, Robotica Spaziale
% Autore: Griguol Francesco m. 2097119
% -------------------------------------------------------------------------

%% INIZIALIZZAZIONE

clear
clc
close all

load("Video_RbS_2024_1.mat")
load("Video_RbS_2024_2.mat")
load("Video_RbS_2024_3.mat")
load("Video_RbS_2024_4.mat")
load("Video_RbS_2024_5.mat")
load("Video_RbS_2024_6.mat")
load("Video_RbS_2024_7.mat")
load("Video_RbS_2024_8.mat")

% Numero di spezzoni del video
numvideo = 8;

% Inizializzazione nuova variabile di video concatenata
video_left = [];
video_right = [];

% Inserisco i video in delle celle per poterli richiamare dopo
videoL = cell(1,numvideo);
videoR = cell(1,numvideo);
for i = 1:numvideo
    % Creazione nomi delle variabili
    left_var_name = sprintf('video_left_%d',i);
    right_var_name = sprintf('video_right_%d',i);
    % Assegnazione dati alle celle 
    videoL{i} = eval(left_var_name);
    videoR{i} = eval(right_var_name);
end

% Concatenazione dei frame di ogni spezzone di video
for i = 1:numvideo
    video_left = cat(3,video_left,videoL{i});
    video_right = cat(3,video_right,videoR{i});
end

% Numero di frame da analizzare
numframe = size(video_left,3);

% Cancellazione variabili superflue
clear video_left_1 video_left_2 video_left_3 video_left_4 video_left_5 
clear video_left_6 video_left_7 video_left_8
clear video_right_1 video_right_2 video_right_3 video_right_4 video_right_5
clear video_right_6 video_right_7 video_right_8 videoL videoR i
clear left_var_name right_var_name numvideo

% Posizione iniziale di acquisizione del primo frame
lat = 45.412136916181574;   % [deg]
lon = 11.899638256840994;   % [deg]

% Inclinazione iniziale asse ottico rispetto x verso il basso [deg]
inclX = -18.8;   
% Inclinazione iniziale asse ottico direzione da nord a est rispetto asse verticale [deg]
inclY = 70;

%% RICOSTRUZIONE VIDEO ESPERIENZA

% Visualizzazione del video (play=1 per vederlo, play=0 per NON vederlo) 
play = 0;

if play == 1

    outputDir = 'output_images';
    delete(fullfile(outputDir, '*'));
    frameIndex = 1;
    for j = 1:numframe
         leftFrame = video_left(:,:,j);
         rightFrame = video_right(:,:,j);
         % Salva i frame come immagini
         imwrite(leftFrame, fullfile(outputDir, sprintf('left_%05d.png', frameIndex)));
         imwrite(rightFrame, fullfile(outputDir, sprintf('right_%05d.png', frameIndex)));
         frameIndex = frameIndex+1;
    
    end 
    disp('Immagini salvate con successo!');
    outputDir = 'output_images'; 
    
    outputVideo = VideoWriter('ricostruzioneVideo.avi', 'Uncompressed AVI'); 
    open(outputVideo);
    
    frameIndex = 1; 
    numFrames = frameIndex - 1; 
    while exist(fullfile(outputDir, sprintf('left_%05d.png', frameIndex)), 'file')
        % Caricamento del frame sinistro 
        leftFrame = imread(fullfile(outputDir, sprintf('left_%05d.png', frameIndex)));
        % Scrittura del frame nel video
        writeVideo(outputVideo, leftFrame); 
        frameIndex = frameIndex + 1;
    end
    
    close(outputVideo); 
    disp('Video creato con successo!');
    implay('ricostruzioneVideo.avi');

end

% Cancellazione variabili superflue
clear play

%% PARAMETRI DELLA STEREOCAMERA

% Parametri intrinseci
camIntr_L = cameraIntrinsics([camInfo.left_cam.fx camInfo.left_cam.fy],...
    [camInfo.left_cam.cx camInfo.left_cam.cy],...
    [camInfo.left_cam.height camInfo.left_cam.width]);
camIntr_R = cameraIntrinsics([camInfo.right_cam.fx camInfo.right_cam.fy],...
    [camInfo.right_cam.cx camInfo.right_cam.cy],...
    [camInfo.right_cam.height camInfo.right_cam.width]);

% Parametro estrinseco
poseC_1 = rigidtform3d([0 0 0],-camInfo.t);

% Stereoparameters
stP = stereoParameters(camIntr_L,camIntr_R,poseC_1);

% Cancellazione variabili superflue
clear camInfo

%% PARAMETRI DELL'ALGORITMO E SALVATAGGIO RISULTATI

% Numero massimo di punti considerabili
Nmax = 2e4;

% Preallocazione memoria numero di cornerpoints di Harris
numPointsHarris_L = zeros(numframe,1);
numPointsHarris_R = zeros(numframe,1);
% Preallocazione memoria numero di features di Harris
numFeaturesHarris_L = zeros(numframe,1);
numFeaturesHarris_R = zeros(numframe,1);
% Preallocazione memoria posa della camera
OptPoseHarris = cell(numframe,1);

% Preallocazione memoria numero di SIFTPoints di SIFT
numPointsSIFT_L = zeros(numframe,1);
numPointsSIFT_R = zeros(numframe,1);
% Preallocazione memoria numero di features di SIFT
numFeaturesSIFT_L = zeros(numframe,1);
numFeaturesSIFT_R = zeros(numframe,1);
% Preallocazione memoria posa della camera
OptPoseSIFT = cell(numframe,1);

% Preallocazione memoria numero di ORBpoints di ORB
numPointsORB_L = zeros(numframe,1);
numPointsORB_R = zeros(numframe,1);
% Preallocazione memoria numero di features di ORB
numFeaturesORB_L = zeros(numframe,1);
numFeaturesORB_R = zeros(numframe,1);
% Preallocazione memoria posa della camera
OptPoseORB = cell(numframe,1);

%% ALGORITMO DI VISUAL ODOMETRY HARRIS

% Calcolo del tempo impiegato per analizzare i video
tStart = tic;
tic

for j = 1:numframe-2
        
    % DETECTOR HARRIS FRAME J
       
    % Cornerpoints immagine sinistra
    PointsHarris_L = detectHarrisFeatures(video_left(:,:,j));
    % Cornerpoints immagine destra
    PointsHarris_R = detectHarrisFeatures(video_right(:,:,j));
    % Selezione dei cornerpoints con la metrica migliore immagine sinistra
    PointsHarris_L = selectStrongest(PointsHarris_L,Nmax);
    % Selezione dei cornerpoints con la metrica migliore immagine destra
    PointsHarris_R = selectStrongest(PointsHarris_R,Nmax);
    
    % DESCRIPTOR FREAK
    
    % Features immagine sinistra
    [FeaturesHarris_L,vPointsHarris_L] = extractFeatures(video_left(:,:,j),PointsHarris_L,Method="FREAK");
    % Features immagine destra
    [FeaturesHarris_R,vPointsHarris_R] = extractFeatures(video_right(:,:,j),PointsHarris_R,Method="FREAK");

    % Memorizza il numero di punti e di features Harris validi
    numPointsHarris_L(j,1) = vPointsHarris_L.Count;
    numFeaturesHarris_L(j,1) = FeaturesHarris_L.NumFeatures;
    numPointsHarris_R(j,1) = vPointsHarris_L.Count;
    numFeaturesHarris_R(j,1) = FeaturesHarris_R.NumFeatures;
       
    % GRAFICI IMMAGINE DESTRA E SINISTRA

    if j == 400

       figure()
       imshow(video_left(:,:,j));
       hold on;
       plot(vPointsHarris_L.selectStrongest(2000));
       title('Punti Harris','FontSize',20,'FontWeight','bold')
       subtitle('Immagine sinistra video 4 frame 62')
       figure()
       imshow(video_right(:,:,j));
       hold on;
       plot(vPointsHarris_R.selectStrongest(2000));
       title('Punti Harris','FontSize',20,'FontWeight','bold')
       subtitle('Immagine sinistra video 4 frame 62')

    end
    
    % MATCHING IMMAGINE SINISTRA E DESTRA POSIZIONE J
        
    % Indice punti di match
    [mindex,~] = matchFeatures(FeaturesHarris_L,FeaturesHarris_R,"Unique",true,"MatchThreshold",10);
    % Cornerpoint con match immagine sinistra
    mPointsHarris_L = vPointsHarris_L(mindex(:,1));
    % Features con match immagine sinistra
    mFeaturesHarris_L = binaryFeatures(FeaturesHarris_L.Features(mindex(:,1),:));
    % Cornerpoints con match immagine destra
    mPointsHarris_R = vPointsHarris_R(mindex(:,2));
    
    % Selezione dei match validi
    YdistHarris = abs(mPointsHarris_L.Location(:,2)-mPointsHarris_R.Location(:,2));
    XdistHarris = abs(mPointsHarris_L.Location(:,1)-mPointsHarris_R.Location(:,1));
    is_valido_H = (YdistHarris < 10) & (XdistHarris < 100) & (XdistHarris > 3);
    mPointsHarris_L = mPointsHarris_L(is_valido_H);
    mFeaturesHarris_L = binaryFeatures(mFeaturesHarris_L.Features(is_valido_H,:));
    mPointsHarris_R = mPointsHarris_R(is_valido_H);

    % GRAFICI MATCHING DESTRO E SINISTRO
        
    if j == 400

       figure() 
       % argomento 5: "falsecolor" (default) | "blend" | "montage"
       showMatchedFeatures(video_left(:,:,j),video_right(:,:,j),mPointsHarris_L,mPointsHarris_R,"falsecolor");
       title('Match Harris','FontSize',20,'FontWeight','bold')
       subtitle('Immagine sinistra video 4 frame 62')

     end
    
     % TRIANGOLAZIONE
    
     Points3dHarris = triangulate(mPointsHarris_L,mPointsHarris_R,stP);

     % GRAFICI PUNTI 3D

     if j == 400

            figure()
            pcshow(Points3dHarris)
            title("Punti 3D Harris",'FontSize',20,'FontWeight','bold')
            subtitle('Video 4 frame 62')
            xlabel("X",'FontSize',15)
            ylabel("Y","FontSize",15)
            zlabel("Z",'FontSize',15)
            view(0,0)

     end
    
     % DETECTOR HARRIS FRAME J+1
    
     % Cornerpoints immagine sinistra
     PointsHarris_L = detectHarrisFeatures(video_left(:,:,j+1));
     % Selezione dei cornerpoints con la metrica migliore immagine sinistra
     PointsHarris_L = selectStrongest(PointsHarris_L,Nmax);
    
     % DESCRIPTOR FREAK FRAME J+1
    
     % Features immagine sinistra
     [FeaturesHarris_L,vPointsHarris_L] = extractFeatures(video_left(:,:,j+1),PointsHarris_L,Method="FREAK");
       
     % MATCHING IMMAGINE SINISTRA FRAME J E IMMAGINE SINISTRA FRAME J+1
        
     % Indice punti di match
     [msindex,~] = matchFeatures(mFeaturesHarris_L,FeaturesHarris_L,"Unique",true,"MatchThreshold",10);
     % mPointsHarris_L = mPointsHarris_L(msindex(:,1));
     jPoints3dHarris = Points3dHarris(msindex(:,1),:); 
     mjPointsHarris_L = vPointsHarris_L(msindex(:,2));
    
     % STIMA ROTOTRASLAZIONE TRA POSIZIONE J E POSIZIONE J+1
    
     [PoseHarris, inlierHarris, ~] = estworldpose(...
     mjPointsHarris_L.Location, jPoints3dHarris, camIntr_L, ...
     'Confidence', 99, 'MaxReprojectionError', 1, 'MaxNumTrials', 1000);
     % Verifica dei dati a disposizione
     jPoints3dHarris_inliers = jPoints3dHarris(inlierHarris,:);
     if isempty(jPoints3dHarris_inliers)
         error('jPoints3dHarris_inliers is empty!');
     end
     if ~isfloat(jPoints3dHarris_inliers)
         error('jPoints3dHarris_inliers is not a floating-point array!');
     end
     if any(isnan(jPoints3dHarris_inliers) | isinf(jPoints3dHarris_inliers))
         error('jPoints3dHarris_inliers contains NaN or Inf values!');
     end
    
     % OTTIMIZZAZIONE NON LINEARE
     OptPoseHarris{j} = bundleAdjustmentMotion(jPoints3dHarris(inlierHarris,:), ...
     mjPointsHarris_L.Location(inlierHarris,:), PoseHarris, camIntr_L,...
     'MaxIteration', 50, 'AbsoluteTolerance', 1e-7,...
     'RelativeTolerance', 1e-15, 'PointsUndistorted', true,...
     'Verbose',true);
    
end

% ULTIMO FRAME VISIBILE
% Cornerpoints immagine sinistra
PointsHarris_L = detectHarrisFeatures(video_left(:,:,end-1));
% Cornerpoints immagine destra
PointsHarris_R = detectHarrisFeatures(video_right(:,:,end-1));
% Selezione dei cornerpoints con la metrica migliore immagine sinistra
PointsHarris_L = selectStrongest(PointsHarris_L,Nmax);
% Selezione dei cornerpoints con la metrica migliore immagine destra
PointsHarris_R = selectStrongest(PointsHarris_R,Nmax);
% Features immagine sinistra
[FeaturesHarris_L,vPointsHarris_L] = extractFeatures(video_left(:,:,end-1),PointsHarris_L,Method="FREAK");
% Features immagine destra
[FeaturesHarris_R,vPointsHarris_R] = extractFeatures(video_right(:,:,end-1),PointsHarris_R,Method="FREAK");
% Memorizza il numero di punti e di features Harris validi
numPointsHarris_L(end-1,1) = vPointsHarris_L.Count;
numFeaturesHarris_L(end-1,1) = FeaturesHarris_L.NumFeatures;
% Indice punti di match
[mindex,~] = matchFeatures(FeaturesHarris_L,FeaturesHarris_R,"Unique",true,"MatchThreshold",10);
% Cornerpoint con match immagine sinistra
mPointsHarris_L = vPointsHarris_L(mindex(:,1));
% Features con match immagine sinistra
mFeaturesHarris_L = binaryFeatures(FeaturesHarris_L.Features(mindex(:,1),:));
% Cornerpoints con match immagine destra
mPointsHarris_R = vPointsHarris_R(mindex(:,2));
% Selezione dei match validi
YdistHarris = abs(mPointsHarris_L.Location(:,2)-mPointsHarris_R.Location(:,2));
XdistHarris = abs(mPointsHarris_L.Location(:,1)-mPointsHarris_R.Location(:,1));
is_valido_H = (YdistHarris < 10) & (XdistHarris < 100) & (XdistHarris > 3);
mPointsHarris_L = mPointsHarris_L(is_valido_H);
mFeaturesHarris_L = binaryFeatures(mFeaturesHarris_L.Features(is_valido_H,:));
mPointsHarris_R = mPointsHarris_R(is_valido_H);
% Punti 3d Harris
Points3dHarris = triangulate(mPointsHarris_L,mPointsHarris_R,stP);

clc
tEndHarris = toc(tStart);

% Cancellazione variabili superflue
clear PointsHarris_L PointsHarris_L FeaturesHarris_L FeaturesHarris_R
clear vPointsHarris_L vPointsHarris_R mindex mPointsHarris_L mPointsHarris_R
clear mFeaturesHarris_L YdistHarris XdistHarris is_valido_H Points3dHarris

%% ALGORITMO DI VISUAL ODOMETRY SIFT

% Calcolo del tempo impiegato per analizzare i singoli video
tStart = tic;
tic

for j = 1:numframe-2
  
    % DETECTOR SIFT FRAME J
        
    % SIFTPoints immagine sinistra
    PointsSIFT_L = detectSIFTFeatures(video_left(:,:,j));
    % SIFTPoints immagine destra
    PointsSIFT_R = detectSIFTFeatures(video_right(:,:,j));
    % Selezione degli SIFTPoints con la metrica migliore immagine sinistra
    PointsSIFT_L = selectStrongest(PointsSIFT_L,Nmax);
    % Selezione degli SIFTPoints con la metrica migliore immagine destra
    PointsSIFT_R = selectStrongest(PointsSIFT_R,Nmax);
    
    % DESCRIPTOR SIFT FRAME J
    
    % Features immagine sinistra
    [FeaturesSIFT_L,vPointsSIFT_L] = extractFeatures(video_left(:,:,j),PointsSIFT_L,Method="SIFT");
    % Features immagine destra
    [FeaturesSIFT_R,vPointsSIFT_R] = extractFeatures(video_right(:,:,j),PointsSIFT_R,Method="SIFT");

    % Memorizza il numero di punti SIFT validi
    numPointsSIFT_L(j,1) = vPointsSIFT_L.Count;
    numFeaturesSIFT_L(j,1) = size(FeaturesSIFT_L,1);
       
    % GRAFICI IMMAGINE DESTRA E SINISTRA 

    if  j == 400

            figure()
            imshow(video_left(:,:,j));
            hold on;
            plot(vPointsSIFT_L.selectStrongest(2000),ShowOrientation=true);
            title('Punti SIFT','FontSize',20,'FontWeight','bold')
            subtitle('Immagine sinistra frame 75')
            figure()
            imshow(video_right(:,:,j));
            hold on;
            plot(vPointsSIFT_R.selectStrongest(2000),ShowOrientation=true);
            title('Punti SIFT','FontSize',20,'FontWeight','bold')
            subtitle('Immagine sinistra frame 75')

    end
    
    % MATCHING IMMAGINE SINISTRA E DESTRA POSIZIONE J
        
    % Indice punti di match
    [mindex,~] = matchFeatures(FeaturesSIFT_L,FeaturesSIFT_R,"Unique",true,"MatchThreshold",0.7);
    % SIFTPoint con match immagine sinistra
    mPointsSIFT_L = vPointsSIFT_L(mindex(:,1));
    % features con match immagine sinistra
    mFeaturesSIFT_L = FeaturesSIFT_L(mindex(:,1),:);
    % SIFTPoints con match immagine destra
    mPointsSIFT_R = vPointsSIFT_R(mindex(:,2));
    
    % Selezione dei match validi
    YdistSIFT = abs(mPointsSIFT_L.Location(:,2)-mPointsSIFT_R.Location(:,2));
    XdistSIFT = abs(mPointsSIFT_L.Location(:,1)-mPointsSIFT_R.Location(:,1));
    is_valido_S = (YdistSIFT < 10) & (XdistSIFT < 100) & (XdistSIFT > 3);
    mPointsSIFT_L = mPointsSIFT_L(is_valido_S);
    mFeaturesSIFT_L = mFeaturesSIFT_L(is_valido_S,:);
    mPointsSIFT_R = mPointsSIFT_R(is_valido_S);

    % GRAFICI MATCHING DESTRO E SINISTRO
        
    if j == 400

            figure() 
            % argomento 5: "falsecolor" (default) | "blend" | "montage"
            showMatchedFeatures(video_left(:,:,j),video_right(:,:,j),mPointsSIFT_L,mPointsSIFT_R,"falsecolor");
            title('Match SIFT','FontSize',20,'FontWeight','bold')
            subtitle('Immagine sinistra video 4 frame 75')

    end
    
    % TRIANGOLAZIONE
    Points3dSIFT = triangulate(mPointsSIFT_L,mPointsSIFT_R,stP);     

    % GRAFICI PUNTI 3D

    if j == 400

            figure()
            pcshow(Points3dSIFT)
            title("Punti 3D SIFT",'FontSize',20,'FontWeight','bold')
            subtitle('Video 4 frame 75')
            xlabel("X",'FontSize',15)
            ylabel("Y",'FontSize',15)
            zlabel("Z",'FontSize',15)
            view(0,0)

    end
    
    % DETECTOR SIFT FRAME J+1
    
    % SIFTPoints immagine sinistra
    PointsSIFT_L = detectSIFTFeatures(video_left(:,:,j+1));
    % Selezione degli SIFTPoints con la metrica migliore immagine sinistra
    PointsSIFT_L = selectStrongest(PointsSIFT_L,Nmax);
    
    % DESCRIPTOR SIFT FRAME J+1
    
    % Features immagine sinistra
    [FeaturesSIFT_L,vPointsSIFT_L] = extractFeatures(video_left(:,:,j+1),PointsSIFT_L,Method="SIFT");
        
    % MATCHING IMMAGINE SINISTRA FRAME J E IMMAGINE SINISTRA FRAME J+1
        
    % Indice punti di match
    [msindex,~] = matchFeatures(mFeaturesSIFT_L,FeaturesSIFT_L,"Unique",true,"MatchThreshold",0.7);
    % mPointsSIFT_L = mPointsSIFT_L(msindex(:,1));
    jPoints3dSIFT = Points3dSIFT(msindex(:,1),:); 
    mjPointsSIFT_L = vPointsSIFT_L(msindex(:,2));
    
    % STIMA ROTOTRASLAZIONE TRA POSIZIONE J E POSIZIONE J+1
    
    [PoseSIFT, inlierSIFT, ~] = estworldpose(...
    mjPointsSIFT_L.Location, jPoints3dSIFT, camIntr_L, ...
    'Confidence', 99, 'MaxReprojectionError', 1, 'MaxNumTrials', 1000);

    % Verifica dei dati a disposizione
    jPoints3dSIFT_inliers = jPoints3dSIFT(inlierSIFT,:);
    if isempty(jPoints3dSIFT_inliers)
       error('jPoints3dSIFT_inliers is empty!');
    end
    if ~isfloat(jPoints3dSIFT_inliers)
       error('jPoints3dSIFT_inliers is not a floating-point array!');
    end
    if any(isnan(jPoints3dSIFT_inliers) | isinf(jPoints3dSIFT_inliers))
       error('jPoints3dSIFT_inliers contains NaN or Inf values!');
    end
    
    % OTTIMIZZAZIONE NON LINEARE
    OptPoseSIFT{j} = bundleAdjustmentMotion(jPoints3dSIFT(inlierSIFT,:), ...
    mjPointsSIFT_L.Location(inlierSIFT,:), PoseSIFT, camIntr_L,...
    'MaxIteration', 50, 'AbsoluteTolerance', 1e-7,...
    'RelativeTolerance', 1e-15, 'PointsUndistorted', true,...
    'Verbose',true);
    
end

% ANALISI ULTIMO FRAME VISIBILE 
% Detector SIFT
PointsSIFT_L = detectSIFTFeatures(video_left(:,:,end-1));
PointsSIFT_R = detectSIFTFeatures(video_right(:,:,end-1));
PointsSIFT_L = selectStrongest(PointsSIFT_L,Nmax);
PointsSIFT_R = selectStrongest(PointsSIFT_R,Nmax);
% Descriptor SIFT
[FeaturesSIFT_L,vPointsSIFT_L] = extractFeatures(video_left(:,:,end-1),PointsSIFT_L,Method="SIFT");
[FeaturesSIFT_R,vPointsSIFT_R] = extractFeatures(video_right(:,:,end-1),PointsSIFT_R,Method="SIFT");
% Memorizza il numero di punti SIFT validi
numPointsSIFT_L(end,1) = vPointsSIFT_L.Count;
numFeaturesSIFT_L(end,1) = size(FeaturesSIFT_L,1);
% Matching tra immagine sinistra e immagine destra ultimo frame
[mindex,~] = matchFeatures(FeaturesSIFT_L,FeaturesSIFT_R,"Unique",true,"MatchThreshold",0.7);
mPointsSIFT_L = vPointsSIFT_L(mindex(:,1));
mFeaturesSIFT_L = FeaturesSIFT_L(mindex(:,1),:);
mPointsSIFT_R = vPointsSIFT_R(mindex(:,2));
% Selezione dei match validi
YdistSIFT = abs(mPointsSIFT_L.Location(:,2)-mPointsSIFT_R.Location(:,2));
XdistSIFT = abs(mPointsSIFT_L.Location(:,1)-mPointsSIFT_R.Location(:,1));
is_valido_S = (YdistSIFT < 10) & (XdistSIFT < 100) & (XdistSIFT > 3);
mPointsSIFT_L = mPointsSIFT_L(is_valido_S);
mFeaturesSIFT_L = mFeaturesSIFT_L(is_valido_S,:);
mPointsSIFT_R = mPointsSIFT_R(is_valido_S); 
% Punti 3d SIFT
Points3dSIFT = triangulate(mPointsSIFT_L,mPointsSIFT_R,stP);

clc
tEndSIFT = toc(tStart);

% Cancellazione variabili superflue
clear PointsSIFT_L PointsSIFT_L FeaturesSIFT_L FeaturesSIFT_R
clear vPointsSIFT_L vPointsSIFT_R mindex mPointsSIFT_L mPointsSIFT_R
clear mFeaturesSIFT_L YdistSIFT XdistSIFT is_valido_S Points3dSIFT

%% ALGORITMO DI VISUAL ODOMETRY ORB

% Calcolo del tempo impiegato per analizzare i video
tStart = tic;
tic

for j = 1:numframe-2
    
    j 

    % DETECTOR ORB FRAME J
       
    % ORBPoints immagine sinistra
    PointsORB_L = detectORBFeatures(video_left(:,:,j));
    % ORBPoints immagine destra
    PointsORB_R = detectORBFeatures(video_right(:,:,j));
    % Selezione degli ORBPoints con la metrica migliore immagine sinistra
    PointsORB_L = selectStrongest(PointsORB_L,Nmax);
    % Selezione degli ORBPoints con la metrica migliore immagine destra
    PointsORB_R = selectStrongest(PointsORB_R,Nmax);
    
    % DESCRIPTOR ORB FRAME J
    
    % Features immagine sinistra
    [FeaturesORB_L,vPointsORB_L] = extractFeatures(video_left(:,:,j),PointsORB_L,Method="ORB");
    % Features immagine destra
    [FeaturesORB_R,vPointsORB_R] = extractFeatures(video_right(:,:,j),PointsORB_R,Method="ORB");

    % Memorizza il numero di punti e di features ORB validi
    numPointsORB_L(j,1) = vPointsORB_L.Count;
    numFeaturesORB_L(j,1) = FeaturesORB_L.NumFeatures;
       
    % GRAFICI IMMAGINE DESTRA E SINISTRA

    if j == 400

            figure()
            imshow(video_left(:,:,j));
            hold on;
            plot(vPointsORB_L.selectStrongest(2000));
            title('Punti ORB','FontSize',20,'FontWeight','bold')
            subtitle('Immagine sinistra video 4 frame 62')
            figure()
            imshow(video_right(:,:,j));
            hold on;
            plot(vPointsORB_R.selectStrongest(2000));
            title('Punti ORB','FontSize',20,'FontWeight','bold')
            subtitle('Immagine sinistra video 4 frame 62')

    end
    
    % MATCHING IMMAGINE SINISTRA E DESTRA POSIZIONE J
        
    % Indice punti di match
    [mindex,~] = matchFeatures(FeaturesORB_L,FeaturesORB_R,"Unique",true,"MatchThreshold",10);
    % ORBPoint con match immagine sinistra
    mPointsORB_L = vPointsORB_L(mindex(:,1));
    % features con match immagine sinistra
    mFeaturesORB_L = binaryFeatures(FeaturesORB_L.Features(mindex(:,1),:));
    % ORBPoints con match immagine destra
    mPointsORB_R = vPointsORB_R(mindex(:,2));
    
    % Selezione dei match validi
    YdistORB = abs(mPointsORB_L.Location(:,2)-mPointsORB_R.Location(:,2));
    XdistORB = abs(mPointsORB_L.Location(:,1)-mPointsORB_R.Location(:,1));
    is_valido_O = (YdistORB < 10) & (XdistORB < 100) & (XdistORB > 3);
    mPointsORB_L = mPointsORB_L(is_valido_O);
    mFeaturesORB_L = binaryFeatures(mFeaturesORB_L.Features(is_valido_O,:));
    mPointsORB_R = mPointsORB_R(is_valido_O);

    % GRAFICI MATCHING DESTRO E SINISTRO
        
    if j == 400

            figure() 
            % argomento 5: "falsecolor" (default) | "blend" | "montage"
            showMatchedFeatures(video_left(:,:,j),video_right(:,:,j),mPointsORB_L,mPointsORB_R,"blend");
            title('Match ORB','FontSize',20,'FontWeight','bold')
            subtitle('Immagine sinistra video 4 frame 62')

    end
    
    % TRIANGOLAZIONE
    Points3dORB = triangulate(mPointsORB_L,mPointsORB_R,stP);

    % GRAFICI PUNTI 3D

    if j == 400

            figure()
            pcshow(Points3dORB)
            title("Punti 3D ORB",'FontSize',20,'FontWeight','bold')
            subtitle('Video 4 frame 62')
            xlabel("X",'FontSize',15)
            ylabel("Y",'FontSize',15)
            zlabel("Z",'FontSize',15)
            view(0,0)

    end
    
    % DETECTOR ORB FRAME J+1
    
    % ORBPoints immagine sinistra
    PointsORB_L = detectORBFeatures(video_left(:,:,j+1));
    % Selezione degli ORBPoints con la metrica migliore immagine sinistra
    PointsORB_L = selectStrongest(PointsORB_L,Nmax);
    
    % DESCRIPTOR ORB FRAME J+1
    
    % Features immagine sinistra
    [FeaturesORB_L,vPointsORB_L] = extractFeatures(video_left(:,:,j+1),PointsORB_L,Method="ORB");
        
    % MATCHING IMMAGINE SINISTRA FRAME J E IMMAGINE SINISTRA FRAME J+1
        
    % Indice punti di match
    [msindex,~] = matchFeatures(mFeaturesORB_L,FeaturesORB_L,"Unique",true,"MatchThreshold",10);
    % mPointsORB_L = mPointsORB_L(msindex(:,1));
    jPoints3dORB = Points3dORB(msindex(:,1),:); 
    mjPointsORB_L = vPointsORB_L(msindex(:,2));
    
    % STIMA ROTOTRASLAZIONE TRA POSIZIONE J E POSIZIONE J+1
    
    [PoseORB, inlierORB, ~] = estworldpose(...
    mjPointsORB_L.Location, jPoints3dORB, camIntr_L, ...
    'Confidence', 99, 'MaxReprojectionError', 1, 'MaxNumTrials', 1000);

    % Verifica dei dati a disposizione
    jPoints3dORB_inliers = jPoints3dORB(inlierORB,:);
    if isempty(jPoints3dORB_inliers)
        error('jPoints3dORB_inliers is empty!');
    end
    if ~isfloat(jPoints3dORB_inliers)
        error('jPoints3dORB_inliers is not a floating-point array!');
    end
    if any(isnan(jPoints3dORB_inliers) | isinf(jPoints3dORB_inliers))
        error('jPoints3dORB_inliers contains NaN or Inf values!');
    end
    
    % OTTIMIZZAZIONE NON LINEARE
    OptPoseORB{j} = bundleAdjustmentMotion(jPoints3dORB(inlierORB,:), ...
    mjPointsORB_L.Location(inlierORB,:), PoseORB, camIntr_L,...
    'MaxIteration', 50, 'AbsoluteTolerance', 1e-7,...
    'RelativeTolerance', 1e-15, 'PointsUndistorted', true,...
    'Verbose',true);
    
end

% ULTIMO FRAME VISIBILE
% Detector ORB
PointsORB_L = detectORBFeatures(video_left(:,:,end-1));
PointsORB_R = detectORBFeatures(video_right(:,:,end-1));
PointsORB_L = selectStrongest(PointsORB_L,Nmax);
PointsORB_R = selectStrongest(PointsORB_R,Nmax);
% Descriptor ORB
[FeaturesORB_L,vPointsORB_L] = extractFeatures(video_left(:,:,end-1),PointsORB_L,Method="ORB");
[FeaturesORB_R,vPointsORB_R] = extractFeatures(video_right(:,:,end-1),PointsORB_R,Method="ORB");
% Memorizza il numero di punti ORB validi
numPointsORB_L(end,1) = vPointsORB_L.Count;
numFeaturesORB_L(end,1) = FeaturesORB_L.NumFeatures;
% Matching tra immagine sinistra e immagine destra ultimo frame
[mindex,~] = matchFeatures(FeaturesORB_L,FeaturesORB_R,"Unique",true,"MatchThreshold",10);
mPointsORB_L = vPointsORB_L(mindex(:,1));
mFeaturesORB_L = binaryFeatures(FeaturesORB_L.Features(mindex(:,1),:));
mPointsORB_R = vPointsORB_R(mindex(:,2));
% Selezione dei match validi
YdistORB = abs(mPointsORB_L.Location(:,2)-mPointsORB_R.Location(:,2));
XdistORB = abs(mPointsORB_L.Location(:,1)-mPointsORB_R.Location(:,1));
is_valido_O = (YdistORB < 10) & (XdistORB < 100) & (XdistORB > 3);
mPointsORB_L = mPointsORB_L(is_valido_O);
mFeaturesORB_L = binaryFeatures(mFeaturesORB_L.Features(is_valido_O,:));
mPointsORB_R = mPointsORB_R(is_valido_O); 
% Punti 3d ORB
Points3dORB = triangulate(mPointsORB_L,mPointsORB_R,stP);

clc
tEndOrb = toc(tStart);

% Cancellazione variabili superflue
clear PointsORB_L PointsORB_L FeaturesORB_L FeaturesORB_R
clear vPointsORB_L vPointsORB_R mindex mPointsORB_L mPointsORB_R
clear mFeaturesORB_L YdistORB XdistORB is_valido_O Points3dORB

%% STAMPA A VIDEO DEI RISULTATI DELL'ALGORITMO

% Tempo di esecuzione dell'algoritmo di visual odometry per ogni metodo
fprintf('TEMPO DI ESECUZIONE ALGORITO DI VISUAL ODOMETRY:\n');
fprintf('------------------------------------------------\n');
fprintf('Metodo di Harris: %2.2f minuti\n', tEndHarris/60);
fprintf('Metodo SIFT: %2.2f ore\n', tEndSIFT/3600);
fprintf('Metodo ORB: %2.2f ore\n', tEndORB/3600);
fprintf('------------------------------------------------\n\n');

% Keypoints per frame medi camera sinistra
fprintf('MEDIA KEYPOINTS PER FRAME - IMMAGINE SINISTRA :\n');
fprintf('--------------------------------------------------\n');
fprintf('Metodo di Harris: %5.0f cornerpoints (Nmax: %5.0f)\n', mean(numPointsHarris_L), Nmax);
fprintf('Metodo SIFT: %5.0f SIFT points (Nmax: %5.0f)\n', mean(numPointsSIFT_L), Nmax);
fprintf('Metodo ORB: %5.0f ORB points (Nmax: %5.0f)\n', mean(numPointsORB_L), Nmax);
fprintf('--------------------------------------------------\n\n');

% Keypoints per frame medi camera destra
fprintf('MEDIA KEYPOINTS PER FRAME - IMMAGINE DESTRA :\n');
fprintf('--------------------------------------------------\n');
fprintf('Metodo di Harris: %5.0f cornerpoints (Nmax: %5.0f)\n', mean(numPointsHarris_R), Nmax);
fprintf('Metodo SIFT: %5.0f SIFT points (Nmax: %5.0f)\n', mean(numPointsSIFT_R), Nmax);
fprintf('Metodo ORB: %5.0f ORB points (Nmax: %5.0f)\n', mean(numPointsORB_R), Nmax);
fprintf('--------------------------------------------------\n\n');

% Feature per frame medi camera sinistra
fprintf('MEDIA FEATURE PER FRAME - IMMAGINE SINISTRA :\n');
fprintf('----------------------------------------------\n');
fprintf('Metodo di Harris: %5.0f features (Nmax: %5.0f)\n', mean(numFeaturesHarris_L), Nmax);
fprintf('Metodo SIFT: %5.0f features (Nmax: %5.0f)\n', mean(numFeaturesSIFT_L), Nmax);
fprintf('Metodo ORB: %5.0f features (Nmax: %5.0f)\n', mean(numFeaturesORB_L), Nmax);
fprintf('----------------------------------------------\n\n');

% Feature per frame medi camera destra
fprintf('MEDIA FEATURE PER FRAME - IMMAGINE DESTRA :\n');
fprintf('----------------------------------------------\n');
fprintf('Metodo di Harris: %5.0f features (Nmax: %5.0f)\n', mean(numFeaturesHarris_R), Nmax);
fprintf('Metodo SIFT: %5.0f features (Nmax: %5.0f)\n', mean(numFeaturesSIFT_R), Nmax);
fprintf('Metodo ORB: %5.0f features (Nmax: %5.0f)\n', mean(numFeaturesORB_R), Nmax);
fprintf('----------------------------------------------\n\n');

%% GRAFICI ANDAMENTO NUMERO DI PUNTI E FEATURES

% Grafico dei punti camera sinistra
figure()
plot(1:numframe-1,numPointsHarris_L(1:end-1,1),"LineWidth",1)
hold on
plot(1:numframe-1,numPointsSIFT_L(1:end-1,1),"LineWidth",1)
plot(1:numframe-1,numPointsORB_L(1:end-1,1),"LineWidth",1)
grid on
title('Confronto numero di keypoints rilevati','FontSize',20,'FontWeight','bold')
subtitle('Immagine sinistra')
xlabel('Numero di frame','FontSize',15)
ylabel('Keypoints','FontSize',15)
xlim([0 795])
ylim([0 Nmax])
xticks([0:100:700 795])
legend('Metodo di Harris','Metodo SIFT','Metodo ORB')

% Grafico dei punti camera destra
figure()
plot(1:numframe-1,numPointsHarris_R(1:end-1,1),"LineWidth",1)
hold on
plot(1:numframe-1,numPointsSIFT_R(1:end-1,1),"LineWidth",1)
plot(1:numframe-1,numPointsORB_R(1:end-1,1),"LineWidth",1)
grid on
title('Confronto numero di keypoints rilevati','FontSize',20,'FontWeight','bold')
subtitle('Immagine destra')
xlabel('Numero di frame','FontSize',15)
ylabel('Keypoints','FontSize',15)
xlim([0 795])
ylim([0 Nmax])
xticks([0:100:700 795])
legend('Metodo di Harris','Metodo SIFT','Metodo ORB')

% Grafico delle features camera sinistra
figure()
plot(1:numframe-1,numFeaturesHarris_L(1:end-1,1),"LineWidth",1)
hold on
plot(1:numframe-1,numFeaturesSIFT_L(1:end-1,1),"LineWidth",1)
plot(1:numframe-1,numFeaturesORB_L(1:end-1,1),"LineWidth",1)
grid on
title('Confronto numero di features rilevate','FontSize',20,'FontWeight','bold')
subtitle('Immagine sinistra')
xlabel('Numero di frame','FontSize',15)
ylabel('Features','FontSize',15)
xlim([0 795])
ylim([0 Nmax])
xticks([0:100:700 795])
legend('Metodo di Harris','Metodo SIFT','Metodo ORB')

% Grafico delle features camera destra
figure()
plot(1:numframe-1,numFeaturesHarris_R(1:end-1,1),"LineWidth",1)
hold on
plot(1:numframe-1,numFeaturesSIFT_R(1:end-1,1),"LineWidth",1)
plot(1:numframe-1,numFeaturesORB_R(1:end-1,1),"LineWidth",1)
grid on
title('Confronto numero di features rilevate','FontSize',20,'FontWeight','bold')
subtitle('Immagine destra')
xlabel('Numero di frame','FontSize',15)
ylabel('Features','FontSize',15)
xlim([0 795])
ylim([0 Nmax])
xticks([0:100:700 795])
legend('Metodo di Harris','Metodo SIFT','Metodo ORB')

%% SISTEMA DI RIFERIMENTO CAMERA

% Function per visualizzare il sistema di riferimento ruotato
% e trovare la matrice di rotazione R che compensa le rotazioni 
R = ReferenceSystem(inclX,inclY);

%% RICOSTRUZIONE TRAIETTORIA E CALCOLO DISTANZA FINALE

% Definizione della posa iniziale
Origine = rigidtform3d(eye(3),[0 0 0]);

% Allocazione memoria posizioni
PositionHarris = zeros(numframe,3);
PositionHarris(1,:) = Origine.Translation;
PositionSIFT = zeros(numframe,3);
PositionSIFT(1,:) = Origine.Translation;
PositionORB = zeros(numframe,3);
PositionORB(1,:) = Origine.Translation;

% Inizializzazione rotazione globale
globalRotationHarris = eye(3);
globalRotationSIFT = eye(3);
globalRotationORB = eye(3);

% Indice di iterazione
indice = 2;

% Aggiornamento della posa
for j = 1:numframe-2

     % Recupero l'informazione di traslazione
     relTranslationHarris = OptPoseHarris{j}.Translation;
     relTranslationSIFT = OptPoseHarris{j}.Translation;
     relTranslationORB = OptPoseHarris{j}.Translation;
          
     % Applico la compensazione di inclinazione alla traslazione
     relTranslationHarris = (R*relTranslationHarris')';
     relTranslationSIFT = (R*relTranslationSIFT')';
     relTranslationORB = (R*relTranslationORB')';

     % Recupero l'informazione di rotazione
     relRotationHarris = OptPoseHarris{j}.Rotation;
     relRotationSIFT = OptPoseHarris{j}.Rotation;
     relRotationORB = OptPoseHarris{j}.Rotation;

     % Aggiornamento della rotazione globale
     globalRotationHarris = globalRotationHarris*relRotationHarris;
     globalRotationSIFT = globalRotationSIFT*relRotationSIFT;
     globalRotationORB = globalRotationORB*relRotationORB;
     
     % Calcolo della nuova posizione
     PositionHarris(indice,:) = PositionHarris(indice-1,:)+(globalRotationHarris*relTranslationHarris')';
     PositionSIFT(indice,:) = PositionSIFT(indice-1,:)+(globalRotationSIFT*relTranslationSIFT')';
     PositionHarris(indice,:) = PositionHarris(indice-1,:)+(globalRotationSIFT*relTranslationSIFT')';
     
     % Aggiornamento dell'indice di salvataggio
     indice = indice+1;     
    
 end

% Visualizzazione della traiettoria nel piano orizzontale XZ
figure()
plot(PositionHarris(:,1).*1e-3,PositionHarris(:,3).*1e-3,'o-','LineWidth',1)
hold on
plot(PositionSIFT(:,1).*1e-3,PositionSIFT(:,3).*1e-3,'o-','LineWidth',1)
plot(PositionORB(:,1).*1e-3,PositionORB(:,3).*1e-3,'o-','LineWidth',1)
scatter(PositionHarris(1,1).*1e-3,PositionHarris(1,3).*1e-3,'Marker','pentagram','LineWidth',2)
scatter(PositionHarris(end,1).*1e-3,PositionHarris(end,3).*1e-3,'Marker','pentagram','LineWidth',2)
legend('Metodo di Harris','Metodo SIFT','Metodo ORB','Partenza','Arrivo','Location','best')
title('Traiettoria ricostruita','FontSize',20,'FontWeight','bold')
subtitle('Piano orizzontale XZ')
xlabel('X [m]','FontSize',15)
ylabel('Z [m]','FontSize',15)
axis equal
grid on

% Fattori di conversione per latitudine e longitudine
meters_per_degree_lat = 111000; % metri per grado di latitudine
meters_per_degree_lon = cosd(lat) * 111000; % metri per grado di longitudine 

% Conversione X e Z in coordinate di latitudine e longitudine
latitudineHarris = lat + ((PositionHarris(:,3).*1e-3)/meters_per_degree_lat);
longitudineHarris = lon + ((PositionHarris(:,1).*1e-3)/meters_per_degree_lon);
latitudineSIFT = lat + ((PositionSIFT(:,3).*1e-3)/meters_per_degree_lat);
longitudineSIFT = lon + ((PositionSIFT(:,1).*1e-3)/meters_per_degree_lon);
latitudineORB = lat + ((PositionORB(:,3).*1e-3)/meters_per_degree_lat);
longitudineORB = lon + ((PositionORB(:,1).*1e-3)/meters_per_degree_lon);

% Visualizzazione della traiettoria su mappa con geoplot
figure()
ax = geoaxes;
geoplot(ax,latitudineHarris, longitudineHarris,'r-','LineWidth',1.3);
hold on
geoplot(ax,latitudineSIFT, longitudineSIFT,'y-','LineWidth',1.3);
geoplot(ax,latitudineORB, longitudineORB,'b-','LineWidth',1.3);
geoscatter(ax,latitudineHarris(1),longitudineHarris(1),150,'filled','Marker','pentagram'); 
geoscatter(ax,latitudineHarris(end),longitudineHarris(end),150,'filled','Marker','pentagram');
ax.Basemap = 'satellite';
title('Traiettoria ricostruita su mappa','FontSize',20,'FontWeight','bold');
subtitle('Visualizzazione satellitare')
legend('Metodo di Harris','Metodo SIFT','Metodo ORB','Partenza','Arrivo','Location','best');

%% CALCOLO DISTANZA TRA PUNTO INIZIALE E PUNTO FINALE

% Calcolo distanza finale
distanceHarris = norm(PositionHarris(end,:)-PositionHarris(1,:));
distanceSIFT = norm(PositionSIFT(end,:)-PositionSIFT(1,:));
distanceORB = norm(PositionORB(end,:)-PositionORB(1,:));

% Visualizzazione a video della distanza
fprintf('DISTANZA TRA PUNTO DI PARTENZA E DI ARRIVO :\n');
fprintf('--------------------------------------------\n');
fprintf('Metodo di Harris: %2.3f metri\n',distanceHarris);
fprintf('Metodo SIFT: %2.3f SIFT metri\n',distanceSIFT);
fprintf('Metodo ORB: %2.3f ORB metri\n',distanceORB);
fprintf('--------------------------------------------\n\n');