
% Function per la rotazione del sistema di riferimento XYZ, la sua
% visualizzazione e l'ottenimento delle matrici di rotazione per compensare
% le rotazioni imposte
%
% Input:  - rot1: prima rotazione [deg]
%         - rot2: seconda rotazione [deg]
% Output: - R: matrice che compensa le rotazioni in input e fa ristabilisce
%              gli assi iniziali
% -------------------------------------------------------------------------

function R = ReferenceSystem(rot1,rot2)

% Conversione angoli da gradi a radianti
rot1rad = deg2rad(rot1);
rot2rad = deg2rad(rot2);

% Matrici di rotazione per ruotare il sistema
R1 = [1 0 0;
      0 cos(rot1rad) -sin(rot1rad);
      0 sin(rot1rad) cos(rot1rad)];

R2 = [cos(rot2rad) 0 sin(rot2rad);
      0 1 0;
     -sin(rot2rad) 0 cos(rot2rad)];

% Definizione del sistema di riferimento classico
Xaxis = [1 0 0]';
Yaxis = [0 1 0]';
Zaxis = [0 0 1]';

% Definizione assi dopo prima rotazione
Xrotx = R1*Xaxis;
Yrotx = R1*Yaxis;
Zrotx = R1*Zaxis;

% Definizione del sistema di riferimento ruotato
XaxisRotated = R2*Xrotx;
YaxisRotated = R2*Yrotx;
ZaxisRotated = R2*Zrotx;

% Visualizzazione sistema di riferimento classico
figure()
quiver3(0,0,0,Xaxis(1),Xaxis(2),Xaxis(3),1,'k',"LineWidth",2,"ShowArrowHead","on");
hold on
quiver3(0,0,0,Yaxis(1),Yaxis(2),Yaxis(3),1,'k',"LineWidth",2,"ShowArrowHead","on");
quiver3(0,0,0,Zaxis(1),Zaxis(2),Zaxis(3),1,'k',"LineWidth",2,"ShowArrowHead","on");
hold off
title('Sistema di riferimento della camera corretto','FontSize',20,'FontWeight','bold')
legend('Asse X','Asse Y','Asse Z')
xlabel('X','FontSize',15)
ylabel('Y','FontSize',15)
zlabel('Z','FontSize',15)
xticks([0 1])
yticks([0 1])
zticks([0 1])
axis equal

% Visualizzazione sistema dopo prima rotazione
figure()
quiver3(0,0,0,Xaxis(1),Xaxis(2),Xaxis(3),1,'k',"LineWidth",2,"LineStyle","-","ShowArrowHead","on");
hold on
quiver3(0,0,0,Yaxis(1),Yaxis(2),Yaxis(3),1,'k',"LineWidth",2,"LineStyle","-","ShowArrowHead","on");
quiver3(0,0,0,Zaxis(1),Zaxis(2),Zaxis(3),1,'k',"LineWidth",2,"LineStyle","-","ShowArrowHead","on");
quiver3(0,0,0,Xrotx(1),Xrotx(2),Xrotx(3), 1,'m','LineWidth',2,'ShowArrowHead','on');
quiver3(0,0,0,Yrotx(1),Yrotx(2),Yrotx(3), 1,'m','LineWidth',2,'ShowArrowHead','on');
quiver3(0,0,0,Zrotx(1),Zrotx(2),Zrotx(3), 1,'m','LineWidth',2,'ShowArrowHead','on');
hold off
title('Sistema di riferimento della camera','FontSize',20,'FontWeight','bold')
subtitle('Asse ottico ruotato verso il basso di 18.8°')
legend('Asse X','Asse Y','Asse Z','Asse X ruotato','Asse Y ruotato','Asse Z ruotato')
xlabel('X','FontSize',15)
ylabel('Y','FontSize',15)
zlabel('Z','FontSize',15)
xticks([0 1])
yticks([0 1])
zticks([0 1])
axis equal

% Visualizzazione del sistema dopo la seconda rotazione
figure()
quiver3(0,0,0,Xaxis(1),Xaxis(2),Xaxis(3),1,'k',"LineWidth",2,"LineStyle","-","ShowArrowHead","on");
hold on
quiver3(0,0,0,Yaxis(1),Yaxis(2),Yaxis(3),1,'k',"LineWidth",2,"LineStyle","-","ShowArrowHead","on");
quiver3(0,0,0,Zaxis(1),Zaxis(2),Zaxis(3),1,'k',"LineWidth",2,"LineStyle","-","ShowArrowHead","on");
hold on
quiver3(0,0,0,XaxisRotated(1),XaxisRotated(2),XaxisRotated(3),1,'m','LineWidth',2,"LineStyle","-",'ShowArrowHead','on');
quiver3(0,0,0,YaxisRotated(1),YaxisRotated(2),YaxisRotated(3),1,'m','LineWidth',2,"LineStyle","-","ShowArrowHead","on");
quiver3(0,0,0,ZaxisRotated(1),ZaxisRotated(2),ZaxisRotated(3),1,'m','LineWidth',2,"LineStyle","-","ShowArrowHead","on");
hold off
title('Sistema di riferimento della camera','FontSize',20,'FontWeight','bold')
subtitle('Asse ottico ruotato verso il basso di 18.8° e verso Est di 70°')
legend('Asse X','Asse Y','Asse Z','Asse X ruotato','Asse Y ruotato','Asse Z ruotato')
xlabel('X','FontSize',15)
ylabel('Y','FontSize',15)
zlabel('Z','FontSize',15)
xticks([0 1])
yticks([0 1])
zticks([0 1])
axis equal

%% Matrici di rotazione per allineamenteo del sistema di riferimento

% Matrici di rotazione
R1 = [1 0 0;
      0 cos(rot1rad) sin(rot1rad);
      0 -sin(rot1rad) cos(rot1rad)];

R2 = [cos(rot2rad) 0 -sin(rot2rad);
      0 1 0;
     sin(rot2rad) 0 cos(rot2rad)];

% Matrice di rotazione che compensa le rotazioni in input
R = R1*R2;











end