% Esercitazione Visual Odometry
% Corso di laurea magistrale - Ingegneria Aerospaziale -
% A.A. 2023-2024, Robotica Spaziale
% Autore: Griguol Francesco m. 2097119
% -------------------------------------------------------------------------

%% Inizializzazione

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

% Visualizzazione del video (play=1 per vederlo, play=0 per NON vederlo) 
play = 0;
if play == 1

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

% Preallocazione memoria numero di ORBpoints di ORB
numPointsORB_L = zeros(100,numvideo);
numPointsORB_R = zeros(100,numvideo);
% Preallocazione memoria numero di features di ORB
numFeaturesORB_L = zeros(100,numvideo);
numFeaturesORB_R = zeros(100,numvideo);
% Preallocazione memoria posa della camera
OptPoseORB = cell(numvideo,1);

% Calcolo del tempo impiegato per analizzare i video
tStart = tic;
tic

for i = 1:numvideo

    % Preallocazione memoria posa per ogni frame
    OptPoseORB{i} = cell(numframe(i),1);

    for j = 1:numframe(i)-1

        immagine = i
        iterazione = j
        
        % DETECTOR ORB FRAME J
        
        % ORBPoints immagine sinistra
        PointsORB_L = detectORBFeatures(video_left{i}(:,:,j));
        % ORBPoints immagine destra
        PointsORB_R = detectORBFeatures(video_right{i}(:,:,j));
        % Selezione degli ORBPoints con la metrica migliore immagine sinistra
        PointsORB_L = selectStrongest(PointsORB_L,Nmax);
        % Selezione degli ORBPoints con la metrica migliore immagine destra
        PointsORB_R = selectStrongest(PointsORB_R,Nmax);
    
        % DESCRIPTOR ORB FRAME J
    
        % Features immagine sinistra
        [FeaturesORB_L,vPointsORB_L] = extractFeatures(video_left{i}(:,:,j),PointsORB_L,Method="ORB");
        % Features immagine destra
        [FeaturesORB_R,vPointsORB_R] = extractFeatures(video_right{i}(:,:,j),PointsORB_R,Method="ORB");

        % Memorizza il numero di punti e di features ORB validi
        numPointsORB_L(j,i) = vPointsORB_L.Count;
        numFeaturesORB_L(j,i) = FeaturesORB_L.NumFeatures;
       
        % GRAFICI IMMAGINE DESTRA E SINISTRA

        if i == 3 && j == 55

            figure()
            imshow(video_left{i}(:,:,j));
            hold on;
            plot(vPointsORB_L.selectStrongest(2000));
            title('Punti ORB','FontSize',20,'FontWeight','bold')
            subtitle('Immagine sinistra video 4 frame 62')
            figure()
            imshow(video_right{i}(:,:,j));
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
        
        if i == 3 && j == 55

            figure() 
            % argomento 5: "falsecolor" (default) | "blend" | "montage"
            showMatchedFeatures(video_left{i}(:,:,j),video_right{i}(:,:,j),mPointsORB_L,mPointsORB_R,"blend");
            title('Match ORB','FontSize',20,'FontWeight','bold')
            subtitle('Immagine sinistra video 4 frame 62')

        end
    
        % TRIANGOLAZIONE
    
        Points3dORB = triangulate(mPointsORB_L,mPointsORB_R,stP);

        % GRAFICI PUNTI 3D

        if i == 3 && j == 55

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
        PointsORB_L = detectORBFeatures(video_left{i}(:,:,j+1));
        % Selezione degli ORBPoints con la metrica migliore immagine sinistra
        PointsORB_L = selectStrongest(PointsORB_L,Nmax);
    
        % DESCRIPTOR ORB FRAME J+1
    
        % Features immagine sinistra
        [FeaturesORB_L,vPointsORB_L] = extractFeatures(video_left{i}(:,:,j+1),PointsORB_L,Method="ORB");
        
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
        OptPoseORB{i}{j} = bundleAdjustmentMotion(jPoints3dORB(inlierORB,:), ...
        mjPointsORB_L.Location(inlierORB,:), PoseORB, camIntr_L,...
        'MaxIteration', 50, 'AbsoluteTolerance', 1e-7,...
        'RelativeTolerance', 1e-15, 'PointsUndistorted', true,...
        'Verbose',true);
    
    end

    if i < numvideo
    
        % ANALISI ULTIMO FRAME VIDEO i 
        
        % Detector ORB
        PointsORB_L = detectORBFeatures(video_left{i}(:,:,end));
        PointsORB_R = detectORBFeatures(video_right{i}(:,:,end));
        PointsORB_L = selectStrongest(PointsORB_L,Nmax);
        PointsORB_R = selectStrongest(PointsORB_R,Nmax);
        % Descriptor ORB
        [FeaturesORB_L,vPointsORB_L] = extractFeatures(video_left{i}(:,:,end),PointsORB_L,Method="ORB");
        [FeaturesORB_R,vPointsORB_R] = extractFeatures(video_right{i}(:,:,end),PointsORB_R,Method="ORB");
        % Memorizza il numero di punti ORB validi
        numPointsORB_L(end,i) = vPointsORB_L.Count;
        numFeaturesORB_L(end,i) = FeaturesORB_L.NumFeatures;
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
        % Triangolazione
        Points3dORB = triangulate(mPointsORB_L,mPointsORB_R,stP);
        
        % ANALISI PRIMO FRAME VIDEO i+1
            
        % Detector ORB
        PointsORB_L = detectORBFeatures(video_left{i+1}(:,:,1));
        PointsORB_R = detectORBFeatures(video_right{i+1}(:,:,1));
        PointsORB_L = selectStrongest(PointsORB_L,Nmax);
        PointsORB_R = selectStrongest(PointsORB_R,Nmax);
        % Descriptor ORB
        [FeaturesORB_L,vPointsORB_L] = extractFeatures(video_left{i+1}(:,:,1),PointsORB_L,Method="ORB");
        [FeaturesORB_R,vPointsORB_R] = extractFeatures(video_right{i+1}(:,:,1),PointsORB_R,Method="ORB");
        % Matching tra immagine immagine sinistra ultimo frame e immagine sinistra
        % primo frame del video successivo
        [msindex,~] = matchFeatures(mFeaturesORB_L,FeaturesORB_L,"Unique",true,"MatchThreshold",10);
        jPoints3dORB = Points3dORB(msindex(:,1),:); 
        mjPointsORB_L = vPointsORB_L(msindex(:,2));
        % Stima rototraslazione tra immagine immagine sinistra ultimo frame e
        % immagine sinistra primo frame del video successivo
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
        % Ottimizzazione non lineare tra immagine immagine sinistra ultimo frame e 
        % immagine sinistra primo frame del video successivo
        OptPoseORB{i}{end} = bundleAdjustmentMotion(jPoints3dORB(inlierORB,:), ...
            mjPointsORB_L.Location(inlierORB,:), PoseORB, camIntr_L,...
            'MaxIteration', 50, 'AbsoluteTolerance', 1e-7,...
            'RelativeTolerance', 1e-15, 'PointsUndistorted', true,...
            'Verbose',true);

    else 

        % ANALISI ULTIMO FRAME VISIBILE VIDEO 8
        
        % Detector ORB
        PointsORB_L = detectORBFeatures(video_left{8}(:,:,end-1));
        PointsORB_R = detectORBFeatures(video_right{8}(:,:,end-1));
        PointsORB_L = selectStrongest(PointsORB_L,Nmax);
        PointsORB_R = selectStrongest(PointsORB_R,Nmax);
        % Descriptor ORB
        [FeaturesORB_L,vPointsORB_L] = extractFeatures(video_left{8}(:,:,end-1),PointsORB_L,Method="ORB");
        [FeaturesORB_R,vPointsORB_R] = extractFeatures(video_right{8}(:,:,end-1),PointsORB_R,Method="ORB");
        % Memorizza il numero di punti ORB validi
        numPointsORB_L(end,i) = vPointsORB_L.Count;
        numFeaturesORB_L(end,i) = FeaturesORB_L.NumFeatures;
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
        % Triangolazione
        Points3dORB = triangulate(mPointsORB_L,mPointsORB_R,stP);
        % Stima rototraslazione tra immagine immagine sinistra ultimo frame e
        % immagine destra ultimo frame
        [PoseORB, inlierORB, status] = estworldpose(...
            mPointsORB_L.Location, Points3dORB, camIntr_L, ...
            'Confidence', 99, 'MaxReprojectionError', 1, 'MaxNumTrials', 1000);
        % Ottimizzazione non lineare tra immagine immagine sinistra ultimo frame e 
        % immagine destra ultimo frame
        OptPoseORB{8}{95} = bundleAdjustmentMotion(Points3dORB(inlierORB,:), ...
            mPointsORB_L.Location(inlierORB,:), PoseORB, camIntr_L,...
            'MaxIteration', 50, 'AbsoluteTolerance', 1e-7,...
            'RelativeTolerance', 1e-15, 'PointsUndistorted', true,...
            'Verbose',true);
        
    end

end

clc
tEnd = toc(tStart);
disp(['Tempo di esecuzione del metodo ORB: ',num2str(tEnd/3600),' ore'])

%% GRAFICI ANDAMENTO NUMERO DI PUNTI E FEATURES

% Grafico dei punti
figure()
for i = 1:numvideo
    plot(1:100,numPointsORB_L(:,i),'LineWidth',1)
    hold on
end
legend('video 1','video 2','video 3','video 4','video 5','video 6','video 7','video 8','Location','best')
grid on
title('Andamento numero di Punti - Detector ORB','FontSize',20,'FontWeight','bold')
subtitle('Immagine sinistra')
xlabel('Numero di frame','FontSize',15)
ylabel('ORBPoints','FontSize',15)

% Grafico delle features
figure()
for i = 1:numvideo
    plot(1:100,numFeaturesORB_L(:,i),'LineWidth',1)
    hold on
end
legend('video 1','video 2','video 3','video 4','video 5','video 6','video 7','video 8','Location','best')
grid on
title('Andamento numero di Features - Descriptor ORB','FontSize',20,'FontWeight','bold')
subtitle('Immagine sinistra')
xlabel('Numero di frame','FontSize',15)
ylabel('Features','FontSize',15)

% Grafico dei punti completo senza interruzioni di video
totPointsORB_L = NaN(totalFrame,1);
for i = 1: numvideo
    totPointsORB_L((1+100*(i-1)):(numframe(i)+(i-1)*100),1) = numPointsORB_L(1:numframe(i),i); 
end
figure()
plot(1:totalFrame,totPointsORB_L,"LineWidth",1)
grid on
title('Andamento numero di punti - Detector ORB','FontSize',20,'FontWeight','bold')
subtitle('Immagine sinistra')
xlabel('Numero di frame','FontSize',15)
ylabel('Cornerpoints','FontSize',15)

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
        relTranslation = OptPoseHarris{i}{j}.Translation;
        % Applico la compensazione di inclinazione alla traslazione
        relTranslation = (R*relTranslation')';
        % Recupero l'informazione di rotazione
        relRotation = OptPoseHarris{i}{j}.Rotation;
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
disp(['Distanza tra la posizione iniziale e finale col metodo ORB: ', num2str(distance*1e-3), ' metri']);

% Visualizzazione della traiettoria nel piano orizzontale XZ
figure()
scatter(Position(:,1).*1e-3,Position(:,3).*1e-3,'Marker','.','LineWidth',1.3)
hold on
scatter(Position(1,1).*1e-3,Position(1,3).*1e-3,'Marker','pentagram','LineWidth',2)
scatter(Position(end,1).*1e-3,Position(end,3).*1e-3,'Marker','pentagram','LineWidth',2)
legend('Traiettoria','Partenza','Arrivo','Location','best')
title('Traiettoria ricostruita - Metodo ORB','FontSize',20,'FontWeight','bold')
subtitle('Piano orizzontale XZ')
xlabel('X [m]','FontSize',15)
ylabel('Z [m]','FontSize',15)
axis equal
grid on

% Fattori di conversione per latitudine e longitudine
meters_per_degree_lat = 111000; % metri per grado di latitudine
meters_per_degree_lon = cosd(lat) * 111000; % metri per grado di longitudine 

% Converti X e Z in coordinate di latitudine e longitudine
latitudine = lat + ((Position(:,3).*1e-3)/meters_per_degree_lat);
longitudine = lon + ((Position(:,1).*1e-3)/meters_per_degree_lon);

% Visualizzazione della traiettoria su mappa con geoplot
figure()
ax = geoaxes;
geoplot(ax,latitudine, longitudine,'r-','LineWidth',1.3);
hold on
geoscatter(ax,latitudine(1),longitudine(1),'filled','Marker','pentagram'); 
geoscatter(ax,latitudine(end),longitudine(end),'filled','Marker','pentagram');
ax.Basemap = 'satellite';
title('Traiettoria ricostruita su mappa','FontSize',20,'FontWeight','bold');
subtitle('Metodo ORB')
legend('Traiettoria','Partenza','Arrivo','Location','best');

%% VERIFICA DISTANZA PUNTI CONSECUTIVI

% Grafico distanza tra posizioni consecutive per vedere se ci sono
% discontinuità o salti troppo elevati
diffPositions = sqrt(sum(diff(Position.*1e-1).^2, 2));
figure()
plot(diffPositions, 'o-');
grid on
title('Distanza tra punti successivi','FontSize',20,'FontWeight','bold')
subtitle('Detector Harris')
xlabel('Numero di frame','FontSize',15)
ylabel('Distanza [cm]','FontSize',15)
