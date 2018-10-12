function typegeom = type_geometrie(C1) 
% % Creation des données
% % Chargement du fichier originel dela piste
% fichier_PISTE=load('piste.txt');
% %size(fichier_PISTE)
% fichier_PISTE=fichier_PISTE(1:4350,11);
% 
% C1 = fichier_PISTE';

n = length(C1);
S1 = 0:0.1:(n-1)/10;
               
D1 = zeros(1,n);
g = 9.81;
ds = 0.1;

% Détection des Cercles/LD :
% la courbure ne varie pas => dC/ds = 0

Ct = diff(C1);
nmax = size(C1,2);
I = find(Ct == 0); %%% se pointer sur la LD
clear limC
limC(1) = 1;
ilim = 1;
ii = 1;
debut=1;

% type de géometrie
% 1 = LD,
% 2 = cercle
% 3 = clothoide dec
% 4 = clothoide acc
typegeom = zeros(1,nmax);

while(ii<size(I,2))
    if (debut==0)
        if((I(ii)+1)==I(ii+1))
            ilim=ilim+1;
            limC(ilim)=I(ii);
            debut=1;
        end
    end
    if((I(ii)+1)~=I(ii+1))
        ilim=ilim+1;
        limC(ilim)=I(ii)+1;
        debut=0;
    end
    ii=ii+1;
end

ii = size(limC,2);
Vlim = zeros(1,nmax);
for i = 1:ii-1
    i_d=limC(i);
    i_f=limC(i+1);
    if ((C1(i_d)~=0) & (C1(i_d)==C1(i_f)))
        
        for j=i_d:i_f
           typegeom(j)=2;
        end
    else
        if ((C1(i_d)==0) & (C1(i_d)==C1(i_f)))
            if (C1(i_d+1)==0)
                for j=i_d:i_f
                    typegeom(j)=1;
                end
            end
        end
    end
end
% mise a 1 de la fin du fichier typegeom :
typegeom(limC(ii):nmax)=1;


% Détection des limites clothoides-clothoides :
% Variation de la courbure avec changement de signe de la derivée

Ctt(1:n-2) = Ct(1:n-2).*Ct(2:n-1);
II = find(Ctt<0);
II = II+1;

ii = size(II,2);
for i = 1:ii
    typegeom(II(i))=2;
end

LimCl = II;

% Détection de chaque partie de clothoide, 
% LimC contient les limite ld/ce et LimCL les limites clothoides

%indice courant
i=1;
Limite = zeros(3,30);
ilim = 1;
% le premier element est une ld qui commence à l'indice 1
Limite(1,ilim) = 1;
Limite(3,ilim) = 1;
% creation d'une liste debut fin avec les indices et le type :

while (i<=nmax)
    if (typegeom(i)==0)
        if ((abs(C1(i-1))-abs(C1(i)))<0)
            typegeom(i)=3;
        else
            typegeom(i)=4;
        end
    end
    if (typegeom(i)~=Limite(3,ilim))
        % fin de la section
        Limite(2,ilim)=i-1;
        % création de la section suivante
        ilim = ilim + 1;
        Limite(1,ilim) = i;
        Limite(3,ilim) = typegeom(i);
    end
    i=i+1;
end
Limite(2,ilim) = nmax;            
