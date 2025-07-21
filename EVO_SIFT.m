% Esercitazione Visual Odometry
% Corso di laurea magistrale - Ingegneria Aerospaziale -
% A.A. 2023-2024, Robotica Spaziale, professore M. Pertile
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

% Numero di video da analizzare
numvideo = 8;
% Numero di frame da analizzare
numframe = [100 100 100 100 100 100 100 95];
% Calcolo del numero di frame complessivi
totalFrame = dot(ones(1,numvideo),numframe);
% Inserisco i video in delle celle per poterli richiamare dopo
video_left = cell(1,numvideo);
video_right = cell(1,numvideo);
for i = 1:numvideo
    % Crea i nomi delle variabili dinamicamente
    left_var_name = sprintf('video_left_%d',i);
    right_var_name = sprintf('video_right_%d',i);
    % Assegna i dati alle celle usando eval
    video_left{i} = eval(left_var_name);
    video_right{i} = eval(right_var_name);
end
clear video_left_1 video_left_2 video_left_3 video_left_4 video_left_5 
clear video_left_6 video_left_7 video_left_8
clear video_right_1 video_right_2 video_right_3 video_right_4 video_right_5
clear video_right_6 video_right_7 video_right_8

% Frequenza di acquisizione delle immagini
FrameRate = 15; % [Hz]

% Tempo totale di acquisizione delle immagini [s]
acquisitionTime = totalFrame/FrameRate;

% Posizione iniziale di acquisizione del primo frame
lat = 45.412136916181574;   % [deg]
lon = 11.899638256840994;   % [deg]

% Inclinazione iniziale asse ottico rispetto x verso il basso [deg]
inclX = -18.8;   
% Inclinazione iniziale asse ottico direzione da nord a est rispetto asse verticale [deg]
inclY = 70;

%% RICOSTRUZIONE VIDEO ESPERIENZA

outputDir = 'output_images';
delete(fullfile(outputDir, '*'));
frameIndex = 1;
for i = 1:numvideo
  
    for j = 1:numframe(i)

        leftFrame = video_left{i}(:,:,j);
        rightFrame = video_right{i}(:,:,j);

        % Salva i frame come immagini
        imwrite(leftFrame, fullfile(outputDir, sprintf('left_%05d.png', frameIndex)));
        imwrite(rightFrame, fullfile(outputDir, sprintf('right_%05d.png', frameIndex)));
        frameIndex = frameIndex+1;
    end
end

disp('Immagini salvate con successo!');

outputDir = 'output_images'; 

outputVideo = VideoWriter('ricostruzioneVideo.avi', 'Uncompressed AVI'); 
open(outputVideo);

frameIndex = 1; 
numFrames = frameIndex - 1; 
while exist(fullfile(outputDir, sprintf('left_%05d.png', frameIndex)), 'file')
    % Carica il frame sinistro 
    leftFrame = imread(fullfile(outputDir, sprintf('left_%05d.png', frameIndex)));

    % Scrivi il frame nel video
    writeVideo(outputVideo, leftFrame); 

    frameIndex = frameIndex + 1;
end

close(outputVideo); 

disp('Video creato con successo!');

% Visualizzazione del video (play=1 per vederlo, play=0 per NON vederlo) 
play = 0;
if play == 1
    implay('ricostruzioneVideo.avi');
end

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

%% ALGORITMO DI VISUAL ODOMETRY

% Numero massimo di punti considerabili 
Nmax = 10e4;

% Preallocazione memoria numero di SIFTPoints di SIFT
numPointsSIFT_L = zeros(100,numvideo);
numPointsSIFT_R = zeros(100,numvideo);
% Preallocazione memoria numero di features di SIFT
numFeaturesSIFT_L = zeros(100,numvideo);
numFeaturesSIFT_R = zeros(100,numvideo);
% Preallocazione memoria posa della camera
OptPoseSIFT = cell(numvideo,1);

% Calcolo del tempo impiegato per analizzare i singoli video
tStart = tic;
tic

for i = 1:numvideo

    % Preallocazione memoria posa per ogni frame
    OptPoseSIFT{i} = cell(numframe(i),1);

    for j = 1:numframe(i)-1

        immagine = i
        iterazione = j
    
        % DETECTOR SIFT FRAME J
        
        % SIFTPoints immagine sinistra
        PointsSIFT_L = detectSIFTFeatures(video_left{i}(:,:,j));
        % SIFTPoints immagine destra
        PointsSIFT_R = detectSIFTFeatures(video_right{i}(:,:,j));
        % Selezione degli SIFTPoints con la metrica migliore immagine sinistra
        PointsSIFT_L = selectStrongest(PointsSIFT_L,Nmax);
        % Selezione degli SIFTPoints con la metrica migliore immagine destra
        PointsSIFT_R = selectStrongest(PointsSIFT_R,Nmax);
    
        % DESCRIPTOR SIFT FRAME J
    
        % Features immagine sinistra
        [FeaturesSIFT_L,vPointsSIFT_L] = extractFeatures(video_left{i}(:,:,j),PointsSIFT_L,Method="SIFT");
        % Features immagine destra
        [FeaturesSIFT_R,vPointsSIFT_R] = extractFeatures(video_right{i}(:,:,j),PointsSIFT_R,Method="SIFT");

        % Memorizza il numero di punti SIFT validi
        numPointsSIFT_L(j,i) = vPointsSIFT_L.Count;
        numFeaturesSIFT_L(j,i) = size(FeaturesSIFT_L,1);
       
        % GRAFICI IMMAGINE DESTRA E SINISTRA 

        if i == 3 && j == 55

            figure()
            imshow(video_left{i}(:,:,j));
            hold on;
            plot(vPointsSIFT_L.selectStrongest(2000),ShowOrientation=true);
            title('Punti SIFT','FontSize',20,'FontWeight','bold')
            subtitle('Immagine sinistra video 4 frame 75')
            figure()
            imshow(video_right{i}(:,:,j));
            hold on;
            plot(vPointsSIFT_R.selectStrongest(2000),ShowOrientation=true);
            title('Punti SIFT','FontSize',20,'FontWeight','bold')
            subtitle('Immagine sinistra video 4 frame 75')

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
        
        if i == 3 && j == 55

            figure() 
            % argomento 5: "falsecolor" (default) | "blend" | "montage"
            showMatchedFeatures(video_left{i}(:,:,j),video_right{i}(:,:,j),mPointsSIFT_L,mPointsSIFT_R,"falsecolor");
            title('Match SIFT','FontSize',20,'FontWeight','bold')
            subtitle('Immagine sinistra video 4 frame 75')

        end
    
        % TRIANGOLAZIONE
    
        Points3dSIFT = triangulate(mPointsSIFT_L,mPointsSIFT_R,stP);     

        % GRAFICI PUNTI 3D

        if i == 3 && j == 55

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
        PointsSIFT_L = detectSIFTFeatures(video_left{i}(:,:,j+1));
        % Selezione degli SIFTPoints con la metrica migliore immagine sinistra
        PointsSIFT_L = selectStrongest(PointsSIFT_L,Nmax);
    
        % DESCRIPTOR SIFT FRAME J+1
    
        % Features immagine sinistra
        [FeaturesSIFT_L,vPointsSIFT_L] = extractFeatures(video_left{i}(:,:,j+1),PointsSIFT_L,Method="SIFT");
        
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
        OptPoseSIFT{i}{j} = bundleAdjustmentMotion(jPoints3dSIFT(inlierSIFT,:), ...
        mjPointsSIFT_L.Location(inlierSIFT,:), PoseSIFT, camIntr_L,...
        'MaxIteration', 50, 'AbsoluteTolerance', 1e-7,...
        'RelativeTolerance', 1e-15, 'PointsUndistorted', true,...
        'Verbose',true);
    
    end

    if i < numvideo
    
        % ANALISI ULTIMO FRAME VIDEO i 
        
        % Detector SIFT
        PointsSIFT_L = detectSIFTFeatures(video_left{i}(:,:,end));
        PointsSIFT_R = detectSIFTFeatures(video_right{i}(:,:,end));
        PointsSIFT_L = selectStrongest(PointsSIFT_L,Nmax);
        PointsSIFT_R = selectStrongest(PointsSIFT_R,Nmax);
        % Descriptor SIFT
        [FeaturesSIFT_L,vPointsSIFT_L] = extractFeatures(video_left{i}(:,:,end),PointsSIFT_L,Method="SIFT");
        [FeaturesSIFT_R,vPointsSIFT_R] = extractFeatures(video_right{i}(:,:,end),PointsSIFT_R,Method="SIFT");
        % Memorizza il numero di punti SIFT validi
        numPointsSIFT_L(end,i) = vPointsSIFT_L.Count;
        numFeaturesSIFT_L(end,i) = size(FeaturesSIFT_L,1);
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
        % Triangolazione
        Points3dSIFT = triangulate(mPointsSIFT_L,mPointsSIFT_R,stP);
        
        % ANALISI PRIMO FRAME VIDEO i+1
            
        % Detector SIFT
        PointsSIFT_L = detectSIFTFeatures(video_left{i+1}(:,:,1));
        PointsSIFT_R = detectSIFTFeatures(video_right{i+1}(:,:,1));
        PointsSIFT_L = selectStrongest(PointsSIFT_L,Nmax);
        PointsSIFT_R = selectStrongest(PointsSIFT_R,Nmax);
        % Descriptor SIFT
        [FeaturesSIFT_L,vPointsSIFT_L] = extractFeatures(video_left{i+1}(:,:,1),PointsSIFT_L,Method="SIFT");
        [FeaturesSIFT_R,vPointsSIFT_R] = extractFeatures(video_right{i+1}(:,:,1),PointsSIFT_R,Method="SIFT");
        % Matching tra immagine immagine sinistra ultimo frame e immagine sinistra
        % primo frame del video successivo
        [msindex,~] = matchFeatures(mFeaturesSIFT_L,FeaturesSIFT_L,"Unique",true,"MatchThreshold",0.7);
        jPoints3dSIFT = Points3dSIFT(msindex(:,1),:); 
        mjPointsSIFT_L = vPointsSIFT_L(msindex(:,2));
        % Stima rototraslazione tra immagine immagine sinistra ultimo frame e
        % immagine sinistra primo frame del video successivo
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
        % Ottimizzazione non lineare tra immagine immagine sinistra ultimo frame e 
        % immagine sinistra primo frame del video successivo
        OptPoseSIFT{i}{end} = bundleAdjustmentMotion(jPoints3dSIFT(inlierSIFT,:), ...
            mjPointsSIFT_L.Location(inlierSIFT,:), PoseSIFT, camIntr_L,...
            'MaxIteration', 50, 'AbsoluteTolerance', 1e-7,...
            'RelativeTolerance', 1e-15, 'PointsUndistorted', true,...
            'Verbose',true);

    else 

        % ANALISI ULTIMO FRAME VISIBILE VIDEO 8
        
        % Detector SIFT
        PointsSIFT_L = detectSIFTFeatures(video_left{8}(:,:,end-1));
        PointsSIFT_R = detectSIFTFeatures(video_right{8}(:,:,end-1));
        PointsSIFT_L = selectStrongest(PointsSIFT_L,Nmax);
        PointsSIFT_R = selectStrongest(PointsSIFT_R,Nmax);
        % Descriptor SIFT
        [FeaturesSIFT_L,vPointsSIFT_L] = extractFeatures(video_left{8}(:,:,end-1),PointsSIFT_L,Method="SIFT");
        [FeaturesSIFT_R,vPointsSIFT_R] = extractFeatures(video_right{8}(:,:,end-1),PointsSIFT_R,Method="SIFT");
        % Memorizza il numero di punti SIFT validi
        numPointsSIFT_L(end,i) = vPointsSIFT_L.Count;
        numFeaturesSIFT_L(end,i) = size(FeaturesSIFT_L,1);
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
        % Triangolazione
        Points3dSIFT = triangulate(mPointsSIFT_L,mPointsSIFT_R,stP);
        % Stima rototraslazione tra immagine immagine sinistra ultimo frame e
        % immagine destra ultimo frame
        [PoseSIFT, inlierSIFT, ~] = estworldpose(...
            mPointsSIFT_L.Location, Points3dSIFT, camIntr_L, ...
            'Confidence', 99, 'MaxReprojectionError', 1, 'MaxNumTrials', 1000);
        % Ottimizzazione non lineare tra immagine immagine sinistra ultimo frame e 
        % immagine destra ultimo frame
        OptPoseSIFT{8}{95} = bundleAdjustmentMotion(Points3dSIFT(inlierSIFT,:), ...
            mPointsSIFT_L.Location(inlierSIFT,:), PoseSIFT, camIntr_L,...
            'MaxIteration', 50, 'AbsoluteTolerance', 1e-7,...
            'RelativeTolerance', 1e-15, 'PointsUndistorted', true,...
            'Verbose',true);

    end

end

clc
tEnd = toc(tStart);
disp(['Tempo di esecuzione del metodo SIFT: ',num2str(tEnd/3600),' ore'])

%% GRAFICI ANDAMENTO NUMERO DI PUNTI E FEATURES

% Grafico dei punti
figure()
for i = 1:numvideo
    plot(1:100,numPointsSIFT_L(:,i),'LineWidth',1)
    hold on
end
legend('video 1','video 2','video 3','video 4','video 5','video 6','video 7','video 8','Location','best')
grid on
title('Andamento numero di Punti - Detector SIFT','FontSize',20,'FontWeight','bold')
subtitle('Immagine sinistra')
xlabel('Numero di frame','FontSize',15)
ylabel('SIFTPoints','FontSize',15)

% Grafico delle features
figure()
for i = 1:numvideo
    plot(1:100,numFeaturesSIFT_L(:,i),'LineWidth',1)
    hold on
end
legend('video 1','video 2','video 3','video 4','video 5','video 6','video 7','video 8','Location','best')
grid on
title('Andamento numero di Features - Descriptor SIFT','FontSize',20,'FontWeight','bold')
subtitle('Immagine sinistra')
xlabel('Numero di frame','FontSize',15)
ylabel('Features','FontSize',15)

% Grafico dei punti completo senza interruzioni di video
totPointsSIFT_L = NaN(totalFrame,1);
for i = 1: numvideo
    totPointsSIFT_L((1+100*(i-1)):(numframe(i)+(i-1)*100),1) = numPointsSIFT_L(1:numframe(i),i); 
end
figure()
plot(1:totalFrame,totPointsSIFT_L,"LineWidth",1)
grid on
title('Andamento numero di punti - Detector SIFT','FontSize',20,'FontWeight','bold')
subtitle('Immagine sinistra')
xlabel('Numero di frame','FontSize',15)
ylabel('SIFT points','FontSize',15)

%% SISTEMA DI RIFERIMENTO CAMERA

% Uso ReferenceSystem.m per visualizzare il sistema di riferimento ruotato
% e trovare la matrice di rotazione che compensa le rotazioni
R = ReferenceSystem(inclX,inclY);

%% RICOSTRUZIONE TRAIETTORIA E CALCOLO DISTANZA FINALE

% Definizione della posa iniziale
Origine = rigidtform3d(eye(3),[0 0 0]);

% Allocazione memoria posizioni
Position = zeros(totalFrame+1,3);
Position(1,:) = Origine.Translation;

% Inizializzazione rotazione globale
globalRotation = eye(3);

% Indice di iterazione
indice = 2;

% Aggiornamento della posa
for i = 1:numvideo
    for j = 1:numframe(i)
        % Recupero l'informazione di traslazione
        relTranslation = OptPoseSIFT{i}{j}.Translation;
        % Applico la compensazione di inclinazione alla traslazione
        relTranslation = (R*relTranslation')';
        % Recupero l'informazione di rotazione
        relRotation = OptPoseSIFT{i}{j}.Rotation;
        % Aggiornamento della rotazione globale
        globalRotation = globalRotation*relRotation;
        % Calcolo della nuova posizione
        Position(indice,:) = Position(indice-1,:)+(globalRotation*relTranslation')';
        % Aggiornamento dell'indice di salvataggio
        indice = indice+1;
    end
end

% Calcolo distanza finale
distance = norm(Position(end,:)-Position(1,:));
disp(['Distanza tra la posizione iniziale e finale col metodo SIFT: ', num2str(distance*1e-3), ' metri']);

% Visualizzazione della traiettoria nel piano orizzontale XZ
figure()
scatter(Position(:,1).*1e-3,Position(:,3).*1e-3,'Marker','.','LineWidth',1.3)
hold on
scatter(Position(1,1).*1e-3,Position(1,3).*1e-3,'Marker','pentagram','LineWidth',2)
scatter(Position(end,1).*1e-3,Position(end,3).*1e-3,'Marker','pentagram','LineWidth',2)
legend('Traiettoria','Partenza','Arrivo','Location','best')
title('Traiettoria ricostruita - Metodo SIFT','FontSize',20,'FontWeight','bold')
subtitle('Piano orizzontale XZ')
xlabel('X [m]','FontSize',15)
ylabel('Z [m]','FontSize',15)
axis equal
grid on

% Fattori di conversione per latitudine e longitudine
meters_per_degree_lat = 111000; % metri per grado di latitudine
meters_per_degree_lon = cosd(lat) * 111000; % metri per grado di longitudine 

% Conversione X e Z in coordinate di latitudine e longitudine
latitudine = lat + ((Position(:,3).*1e-3)/meters_per_degree_lat);
longitudine = lon + ((Position(:,1).*1e-3)/meters_per_degree_lon);

% Visualizzazione della traiettoria su mappa con geoplot
figure()
ax = geoaxes;
geoplot(ax,latitudine, longitudine,'r-','LineWidth',1.3);
hold on
geoscatter(ax,latitudine(1),longitudine(1),150,'filled','Marker','pentagram'); 
geoscatter(ax,latitudine(end),longitudine(end),150,'filled','Marker','pentagram');
ax.Basemap = 'satellite';
title('Traiettoria ricostruita su mappa','FontSize',20,'FontWeight','bold');
subtitle('Metodo SIFT')
legend('Traiettoria','Partenza','Arrivo','Location','best');

%% VERIFICA DISTANZA PUNTI CONSECUTIVI

% Grafico distanza tra posizioni consecutive per vedere se ci sono
% discontinuità o salti troppo elevati
diffPositions = sqrt(sum(diff(Position.*1e-1).^2, 2));
figure()
plot(diffPositions, 'o-');
grid on
title('Distanza tra punti successivi','FontSize',20,'FontWeight','bold')
subtitle('Detector SIFT')
xlabel('Numero di frame','FontSize',15)
ylabel('Distanza [cm]','FontSize',15)
