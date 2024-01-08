% paramètre pour le dimensionnement
% permet de visualiser le workspace en 2DIMENSIONS
% ainsi que le couple calculé pour chaque point du workspace en trois
% dimension (utiliser l'outil rotation)

% le paramètre L5 est mis a 0 car nous ne mettons pas de bras L5
% a = espace entre les moteurs

clear all
close all
%% Longueur des bielles :
% en m
L1=13e-2;
L2=13e-2;
L3=15e-2;
L4=15e-2;
L5=0e-2;
a=6e-2;



COUPLE2_MAX = 0;
COUPLE1_MAX=0;

theta2_max_couple2=0;
theta1_max_couple2=0;

theta2_max_couple1=0;
theta1_max_couple1=0;

xp_couple_max = 0;
yp_couple_max = 0;

xP_vector=[];
yP_vector=[];

couple1_vector=[];
couple2_vector=[];
indice_i_position=1;

x_max_amplitude=0;
y_max_amplitude=0;

figure(1); hold on; grid on;

  %-----------Force F au point P ----------------------------%
            % En Newton

F_vector= [0 2.5 0];
pas = 0.01;

for theta1 = [0: pas : pi]
    for theta2 = [0 : pas: pi]
        % Paramètres géométriques


        %-----------Coordonnées du point A ----------------------------%

        x_a= -a +L2*cos(theta2);
        y_a= L2*sin(theta2);

        %-----------Coordonnées du point B ----------------------------%

        x_b= a+L1*cos(theta1);
        y_b=L1*sin(theta1);

        AB = sqrt((x_b-x_a).^2 + (y_b-y_a).^2);
        AH = (L3.^2 - L4.^2+AB.^2)/(2*AB);

        %-----------Coordonnées du point H ----------------------------%

        y_h = y_a + (y_b -y_a)*(AH/AB);
        x_h= x_a+(x_b - x_a)*AH/AB;
    
        if L3^2 - AH^2 > 0 % domaine atteignable  
            CH= sqrt(L3^2 - AH^2);

            %-----------Coordonnées du point C ----------------------------%

            x_c= x_h - CH*(y_h-y_a)/AH;
            y_c=y_h+CH*(x_h-x_a)/AH;

            %-----------Coordonnées du point P ----------------------------%
            y_P= (((L4+L5)/(L4))*(y_c-y_b))+y_b;
            x_P= (((L4+L5)/(L4))*(x_c-x_b))+x_b;

%             if y_P > 0
%                 plot(x_P,y_P,'d'); % on ne trace que le dvt du pilote
%             end 


          

            %-----------Parametres  ----------------------------%


            %-----------VECTEUR O2A  ----------------------------%

            x_O2_A = L2*cos(theta2); % O2A = x_O2_A*x + y_O2_A*y
            y_O2_A = L2*sin(theta2);

            O2A_vector =[x_O2_A  y_O2_A  0];



            %-----------VECTEUR BP  ----------------------------%

            x_BP = x_P - x_b; % BP = x_BP*x + y_BP*y
            y_BP = y_P - y_b;

            BP_vector = [x_BP y_BP 0];


            %-----------VECTEUR AC  ----------------------------%

            x_AC = x_c - x_a; % AC = x_AC*x + y_AC*y
            y_AC = y_c - y_a;

            AC = sqrt((x_AC).^2 + (y_AC).^2);

            AC_vector= [x_AC y_AC 0];

            %-----------VECTEUR BC  ----------------------------%

            x_BC = x_c - x_b; % BC = x_BC*x + y_BC*y
            y_BC = y_c - y_b;

            BC_vector=[x_BC y_BC 0];

            %-----------VECTEUR 01B  ----------------------------%

            x_01B = L1*cos(theta1); % 01B = x_01B*x + y_01B*y
            y_01B = L1*sin(theta1);

           % 01B = sqrt((x_01B).^2 + (y_01B).^2);

            O1B_vector = [x_01B y_01B 0];

            %-----------Resultante R23 = R34  ----------------------------%
 
            R23_methode2 = AC*norm(cross(-BP_vector,F_vector)) /norm(cross(BC_vector,AC_vector)); 
            R34= R23_methode2;
    
            % prendre le signe de R34 selon + ou  AC_vector
            
            temp = cross(-BP_vector,F_vector); 
            R34_vector = sign(temp(3))*R23_methode2*AC_vector/AC;

            %----------Calcul des couples  ----------------------------%

            %O2A_vectoriel_AC = x_O2_A*y_AC -y_O2_A*x_AC; % norme selon z
            O2A_vectoriel_AC_methode2 = cross(O2A_vector,AC_vector);
            
            COUPLE2 = R23_methode2* norm(O2A_vectoriel_AC_methode2)/ AC; % en Newton.m

            if COUPLE2 > COUPLE2_MAX 
                COUPLE2_MAX=COUPLE2;
                xp_couple_max = x_P;
                yp_couple_max= y_P;

                theta2_max_couple2=theta2;
                theta1_max_couple2=theta1;
            end

 
            %-----------COUPLE 1 _ methode2---------------%
            R14_vector= -R34_vector-F_vector;

            COUPLE1=norm(cross(O1B_vector, R14_vector));

            if COUPLE1 > COUPLE1_MAX 
                COUPLE1_MAX=COUPLE1;
                
            end

             if y_P > 0 & COUPLE1<2.5 & COUPLE2<2.5 & y_P > 0.1 & (x_b) > abs(x_a)
                    if x_P>x_max_amplitude
                        x_max_amplitude=x_P;
                    end

                    if y_P>y_max_amplitude;
                        y_max_amplitude=y_P;
                    end
                   xP_vector(indice_i_position)=x_P;
                   yP_vector(indice_i_position)=y_P;
                   couple1_vector(indice_i_position)=COUPLE1;
                   couple2_vector(indice_i_position)=COUPLE2;
         
            
                
            end 

            

        end
        indice_i_position=indice_i_position+1;

    end

end 

disp("le couple max est COUPLE 2 =  "+COUPLE2_MAX+" Nm pour theta 1 = "+theta1_max_couple2*180/(pi)+"° et theta2 ="+ theta2_max_couple2*180/(pi));
disp("le couple max est COUPLE 1 = "+COUPLE1_MAX+" Nm");


y_max_amplitude=y_max_amplitude*100;
x_max_amplitude=x_max_amplitude*100;
xP_vector=xP_vector*100;
yP_vector=yP_vector*100;
plot3(xP_vector,yP_vector,couple1_vector,'d','Color','b')
xlabel("x en cm");
ylabel("y en cm")
zlabel("COUPLE 1 EN Nm")

txt = ['amplitude max selon y: ' num2str(y_max_amplitude) ' cm'];
text(-15,5,txt)

txt = ['amplitude max selon x: ' num2str(x_max_amplitude) ' cm'];
text(-15,7,txt)

txt = ['espacement entre les actionneurs : ' num2str(a) ' cm'];
text(-15,9,txt)
title("Couple 1 en fonction de la position")
subtitle(" L1= "+L1+"cm, L2= "+ L2+ " cm , L3 = " +L3 +" cm, L4=" + L4+"cm, L5="+L5 +" cm");

figure(2); hold on; grid on;


plot3(xP_vector,yP_vector,couple2_vector,'d')
xlabel("x en cm");
ylabel("y en cm")
zlabel("COUPLE 2 EN Nm")
title("Couple 2 en fonction de la position")
subtitle(" L1= "+L1+"cm, L2= "+ L2+ " cm , L3 = " +L3 +" cm, L4=" + L4+"cm, L5="+L5 +" cm");


txt = ['amplitude max selon y: ' num2str(y_max_amplitude) ' cm'];
text(-15,5,txt)

txt = ['amplitude max selon x: ' num2str(x_max_amplitude) ' cm'];
text(-15,7,txt)

txt = ['espacement entre les actionneurs : ' num2str(a) ' cm'];
text(-15,9,txt)

