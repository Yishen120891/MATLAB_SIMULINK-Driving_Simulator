function [indice,dist] = matchcarto(xgps,ygps,XGPS,YGPS)


%XGPS : position x de la piste
%YGPS : position y de la piste
%xgps : position x du GPS 
%ygps : position y du GPS

%N = nargin;

P = [xgps;ygps];

Dmax = 3;

% if (N==6)
%     % mapmatching sur une partie de la carto
%     II1 = (indice-Nav):(indice+Nap);
%     IInt = mod(II1-1,length(piste.x))+1;
%     D1 = 1e6;
%     I1 = 0;
%     D2 = 1e6;
%     I2 = 0;
%     for i=IInt
%         P1 = [piste.xcg(i);piste.ycg(i)];
%         P2 = [piste.xcd(i);piste.ycd(i)];
%         Dtmp = norm(P1-P);
%         if (Dtmp<D1)
%             D1 = Dtmp;
%             I1 = i;
%         end
%         Dtmp = norm(P2-P);
%         if (Dtmp<D2)
%             D2 = Dtmp;
%             I2 = i;
%         end
%     end
%     if ((D1>Dmax) & (D2>Dmax))
%         N=3;
%     else
%         if (D1<=D2)
%             II = I1;
%             NVoie = 1;
%             Dmin = D1
%         else
%             II = I2;
%             NVoie = 2;
%             Dmin = D2;
%         end
%     end
% end
N=3;
if (N==3)
    % mapmatching sur la totalité de la piste
    
    D1 = 1e6;
    I1 = 0;
    D2 = 1e6;
    I2 = 0;
    
    for i=1:length(XGPS)
        P1 = [XGPS(i);YGPS(i)];
        %P2 = [piste.xcd(i);piste.ycd(i)];
        Dtmp = norm(P1-P);
        if (Dtmp<D1)
            D1 = Dtmp;
            I1 = i;
        end
%         Dtmp = norm(P2-P);
%         if (Dtmp<D2)
%             D2 = Dtmp;
%             I2 = i;
%         end
    end
    II = I1;
    Dmin = D1;
   
%     if (D1<=D2)
%         II = I1;
%         NVoie = 1;
%         Dmin = D1;
%     else
%         II = I2;
%         NVoie = 2;
%         Dmin = D2;
%     end
end
    
indice = II;
dist = Dmin;
