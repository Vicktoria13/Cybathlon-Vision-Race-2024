#!/usr/bin/env python3

import rclpy
import pyxbox.xbox as xbox
import math
from rclpy.node import Node

from dynamixel_sdk_custom_interfaces.msg import SetPosition
#from std_msgs.msg import Int32

esp = 25

l1 = 13
l2 = 13
l3 = 11
l4 = 11
a = 3

vitesse = 20

#Format floating point number to string format -x.xxx
def fmtFloat(n):
    return '{:6.3f}'.format(n)

#Permet d'avoir les angles en radian en fonction de x et y que l'on veut atteindre

def modele_inverse(x,y):
    dist = math.sqrt(pow((-a+x),2)+pow(y,2))
    alpha = math.acos((math.pow(l1,2)-pow((l4),2)+math.pow(dist,2))/(2*l1*dist))
    beta = math.atan2(y,a-x)
    teta1 = math.pi-(alpha+beta)

    xc= (l4*x)/l4
    yc = (l4*y)/l4
    dist2= math.sqrt(pow((a+xc),2)+pow(yc,2))
    sigma= math.acos((math.pow(l2,2)-math.pow(l3,2)+pow(dist2,2))/(2*l2*dist2))
    gamma = math.atan2(yc,(a+xc))
    teta2 = sigma + gamma
    return (teta1, teta2)


def modele_direct(theta1,theta2):
    xA = -a + l2*math.cos(theta2)
    yA = l2*math.sin(theta2)
    xB = a + l1*math.cos(theta1)
    yB = l1*math.sin(theta1)
    AB = ((xB - xA)**2 + (yB - yA)**2)**0.5
    AH = (l3**2 - l4**2 + AB**2)/(2*AB)

    xH = xA + (AH/AB)*(xB - xA)
    yH = yA + (AH/AB)*(yB - yA)
    CH = (l3**2 - AH**2)**0.5

    xC = xH - (yH - yA)*CH/AH
    yC = yH + (xH - xA)*CH/AH

    x = (l4)*(xC-xB)/l4 + xB
    y = (l4)*(yC-yB)/l4 + yB
    
    return(x,y)

#Retourne les positions en angle (rad)
def pos_to_angle(tup):
    theta1= ((tup[0])/4096)*2*math.pi
    theta2= ((tup[1])/4096)*2*math.pi

    return (theta1,theta2)

#Retourne les angles en position de moteur

def angle_to_pos(tup):
    pos1 = ((tup[0]/(2*math.pi))*4096)
    pos2 = ((tup[1]/(2*math.pi))*4096)

    return (pos1,pos2)

#Permet de retourner la position x et y en fonction de la position du joystick (on a les 8 positions cardinales)

def direction(x,y): #Optimisation possible en faisant moins de if ( if y >= ... des le debut puis faire la liste possible)
    inf=0.3
    sup=3.0
    #Si on veut etre pointilleux, il faudrait avoir une norme constante, sur les valeurs de retour de x et y
    if x**2+y**2>=0.5:
        #Haut
        if y>sup*x and y>-sup*x :
            return (0.0,20.0)
        #Diag haut-droite
        elif sup*x > y and y>inf*x and x>0: 
            return (3.5,18.0)
        #Droite
        elif inf*x>y and y>-inf*x and x>0:
            return (4.0,14.0)
        #Gauche
        elif -inf*x >y and y>inf*x and x<0:
            return (-4.0,14.0)
        #Diag haut-gauche
        elif -inf*x <y and y <-sup*x and x<0:
            return (-3.5,18.0)
        else : #Les autres directions
            return (0.0,15.0)
    else : #Les autres directions
        return (0.0,15.0)
    
#Class du node
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('Manette') #Nom du node
        self.joy_ = xbox.Joystick() #Class Xbox de la manette
        self.xfinal_ = 0.0
        self.yfinal_ = 15.0
        self.posfinal1_ = 1000
        self.posfinal2_ = 700
        self.epsilon_ = esp
        self.present_pos1_=0
        self.present_pos2_=0
        #self.cpt_vibreur_=0

        self.move_ = False
        self.initialisation_ = True
        self.mot1_on_ = False
        self.mot2_on_ = False

        self.set_pos_publisher_setPOS_ = self.create_publisher(SetPosition, 'set_position', 10)
        #self.set_int_publisher_microROS_ = self.create_publisher(Int32, 'microROS', 10)
        self.get_pos_subscriber_ = self.create_subscription(SetPosition,'current_position',self.call_back,10)
        self.get_logger().info("Node Manette started")
        
    def call_back(self, current_msg : SetPosition):
        setPOS_msg = SetPosition()
        #microROS_msg = Int32()
        if self.initialisation_: #Permet au lancement de récupérer les positions des moteurs
            if current_msg.id ==1 :
                self.present_pos1_=current_msg.position-3270
                self.get_logger().info("Set Current Position ID 1")
                self.get_logger().info(f'Publishing: "{current_msg.id} / {self.present_pos1_}"')
            elif current_msg.id == 2:
                self.present_pos2_=current_msg.position-1450
                self.get_logger().info("Set Current Position ID 2")
                self.get_logger().info(f'Publishing: "{current_msg.id} / {self.present_pos2_}"')
            if self.present_pos1_ != 0 and self.present_pos2_!= 0:
                self.get_logger().info("Initialisation Current Position Over")
                self.initialisation_=False
        else :
            if not self.move_ : #Permet de récupérer les positions finales et d'envoyer des vitesses aux moteurs si on veut se déplacer (CAS où on ne bouge pas actuellement)
                
                """if self.cpt_vibreur_ == 50 :
                    microROS_msg.data=2 #Cas où on est dans le else de l'autre code
                    self.set_int_publisher_microROS_.publish(microROS_msg)
                
                self.cpt_vibreur_ +=1
                """
                self.get_logger().info('Acces new valeur')
                (self.xfinal_,self.yfinal_)=direction(self.joy_.leftX(),self.joy_.leftY())
                self.posfinal1_,self.posfinal2_=modele_inverse(self.xfinal_,self.yfinal_)
                self.posfinal1_,self.posfinal2_=angle_to_pos((self.posfinal1_,self.posfinal2_))
                delta1=self.posfinal1_-self.present_pos1_
                delta2=self.posfinal2_-self.present_pos2_
                self.get_logger().info(f'{fmtFloat(self.posfinal1_)}/ {fmtFloat(self.present_pos1_)} / {fmtFloat(delta1)}')
                self.get_logger().info(f'{fmtFloat(self.posfinal2_)}/ {fmtFloat(self.present_pos2_)} / {fmtFloat(delta2)}')
                absdelta1 = math.fabs(delta1)
                absdelta2 = math.fabs(delta2)
                if absdelta1 > absdelta2: #Pour savoir lequel a le plus de distance a parcourir pour pouvoir finir en meme temps
                    if delta1 >0: #Si on doit augmenter notre position du moteur 1
                        setPOS_msg.position=vitesse
                    elif delta1 <0:
                        setPOS_msg.position=-vitesse
                    else :
                        setPOS_msg.position=0
                    if setPOS_msg.position!=0:
                        setPOS_msg.id=1
                        self.mot1_on_ = True
                        self.move_ = True
                        self.set_pos_publisher_setPOS_.publish(setPOS_msg)
                        self.get_logger().info(f'Publishing: "{setPOS_msg.id} / {self.posfinal1_}"')
                    if delta2 >0: #Si on doit augmenter notre position du moteur 2
                        setPOS_msg.position=math.floor(vitesse*absdelta2/absdelta1)
                    elif delta2 < 0:
                        setPOS_msg.position=-math.floor(vitesse*absdelta2/absdelta1)
                    else :
                        setPOS_msg.position=0
                    if setPOS_msg.position!=0:
                        setPOS_msg.id=2
                        self.mot2_on_ = True
                        self.move_ = True
                        self.set_pos_publisher_setPOS_.publish(setPOS_msg)
                        self.get_logger().info(f'Publishing: "{setPOS_msg.id} / {self.posfinal2_}"')
                else : #Le moteur 2 a plus de distance a parcourir
                    if delta1 >0: #Si on doit augmenter la position du moteur 1
                        setPOS_msg.position=math.floor(vitesse*absdelta1/absdelta2)
                    elif delta1 <0:
                        setPOS_msg.position=-math.floor(vitesse*absdelta1/absdelta2)
                    else :
                        setPOS_msg.position=0
                    if setPOS_msg.position!=0:
                        setPOS_msg.id=1
                        self.mot1_on_ = True
                        self.move_ = True
                        self.set_pos_publisher_setPOS_.publish(setPOS_msg)
                        self.get_logger().info(f'Publishing: "{setPOS_msg.id} / {self.posfinal1_}"')
                    if delta2 >0: #Si on doit augmenter la position du moteur 2
                        setPOS_msg.position=vitesse
                    elif delta2 < 0:
                        setPOS_msg.position=-vitesse
                    else :
                        setPOS_msg.position=0
                    if setPOS_msg.position!=0:
                        setPOS_msg.id=2
                        self.mot2_on_ = True
                        self.move_ = True
                        self.set_pos_publisher_setPOS_.publish(setPOS_msg)
                        self.get_logger().info(f'Publishing: "{setPOS_msg.id} / {self.posfinal2_}"')
            else : #Permet de stopper les moteurs à la position voulue (Cas où on se déplace)
                """microROS_msg.data=2 #Cas où on est dans le else de l'autre code
                self.set_int_publisher_microROS_.publish(microROS_msg)
                """
                self.get_logger().info('Acces asservissement')
                if current_msg.id ==1 :
                    self.present_pos1_=current_msg.position-3270 #On est à +3270 sur wizard en position 0, on met le signe opposé car on veut la valeur 0 pour comparer avec le modèle inverse
                    self.get_logger().info(f'Publishing: "{current_msg.id} / {self.present_pos1_}"')
                elif current_msg.id == 2:
                    self.present_pos2_=current_msg.position-1450
                    self.get_logger().info(f'Publishing: "{current_msg.id} / {self.present_pos2_}"')
                #Condition pour arreter le moteur 1
                if (self.mot1_on_) and (self.posfinal1_ - self.epsilon_ <=self.present_pos1_) and (self.posfinal1_ + self.epsilon_ >= self.present_pos1_) :
                    setPOS_msg.id=1
                    setPOS_msg.position=0
                    self.set_pos_publisher_setPOS_.publish(setPOS_msg)
                    self.get_logger().info("Stop Moteur 1")
                    self.mot1_on_ = False
                #Condition pour arreter le moteur 2
                if (self.mot2_on_) and (self.posfinal2_ - self.epsilon_ <=self.present_pos2_) and (self.posfinal2_ + self.epsilon_ >= self.present_pos2_) :
                    setPOS_msg.id=2
                    setPOS_msg.position=0
                    self.set_pos_publisher_setPOS_.publish(setPOS_msg)
                    self.get_logger().info("Stop Moteur 2")
                    self.mot2_on_ = False
                #Si les 2 moteurs ne bougent plus on veut recevoir les nouvelles positions
                if not self.mot1_on_ and not self.mot2_on_ :
                    """if (self.present_pos1_ - self.epsilon_ <= 642) and (self.present_pos1_ + self.epsilon_ >= 642 ) :
                        if (self.present_pos2_ - self.epsilon_ <= 1405) and (self.present_pos2_ + self.epsilon_ >= 1405) :
                            self.cpt_vibreur_= 0
                            microROS_msg.data = 1
                            self.set_int_publisher_microROS_.publish(microROS_msg)
                    else:
                        microROS_msg.data = 0
                        self.set_int_publisher_microROS_.publish(microROS_msg)
                    """
                    self.get_logger().info("Stop Les Moteurs")
                    self.move_ = False
                    self.present_pos1_ = self.posfinal1_
                    self.present_pos2_ = self.posfinal2_


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
