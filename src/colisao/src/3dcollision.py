#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud
from array import array
from math import atan2,degrees,acos,sqrt
from std_msgs .msg import Int32,String,Int16
import numpy as np
from geometry_msgs.msg import Vector3
########################################################################
#Puxa valores do arquivo de parametros do laser
#model = rospy.get_param("model")
#a = rospy.get_param("a")
#d_min = rospy.get_param("d_min") #Usado para filtrar ruidos
#d_max = rospy.get_param("d_max")
#pares = rospy.get_param("pares")
#Distancia dos pontose
# Variavel para manter como colisao enquanto houver qualquer ponto que seja classificado como perigoso
check_collision = 0
#Seleciona os paremtros de distancia minima e max para o modelo em 3d
#if model == "3D":
#  angulo = a
#  dist_min = d_min
#  dist_max = d_max
#  p = pares
########################################################################
# Classifica como colisao e publica
global colisao
colisao = 0
#Colisao antigo
global old_colisao
old_colisao = 0
#Variavel da os setores de colisao
global pontos_colisao
pontos_colisao = 0
#Variavel responsavel por alterar a cor do led para amarelo
global point_min
#Inicia vetor de valores minimos de distancia
point_min = Vector3()
point_min.x = 0.0
point_min.y = 0.0
point_min.z = 0.0

def callback(data):
    global colisao,old_colisao,pontos_colisao,point_min

    #soma pares de pontos com angulo alto
    pares = 0

    #Vetores de pontos
    vetx = np.array([])
    vety = np.array([])
    vetz = np.array([])
    vetaz = np.array([])
    # soma dos quadrados dos pontos
    sum_root_2d = 0
    sum_root_3d = 0
    # contador de nivel
    j = 0
    # incializa variavel nivel
    nivel = 1
    #controle de nivel
    controle_nivel = 0
    # Controle do for
    cont = 0
    # angulo entre pontos
    angle = 0
    # Vetor de comparacao entre variaveis
    comp = np.array([])
    # Comparacao entre Z
    compz = np.array([])
    # Pontos em Z
    ponto_z = 0
    # Filtra dados
    pontos = 0
    #variavel que sinaliza colisao
    check_collision = 0
    #variavel para salvar a distancia em z
    dz = 0
    #variavel para salvar a distancia no plano
    dp = 0
    #varial que altera a cor do led
    green = 0
    yellow = 0
    red = 0
    #variavel para salvar a que distnacia do robo um angulo maior que 30 foi detectado
    d2D = 0
    #Altura do ponto
    height = 0
    #Valores dos pontos mais proximos em x,y e z
    p_minx = np.array([])
    p_miny = np.array([])
    p_minz = np.array([])
    for i in range (0,len(data.points)):

        #Vetores que salvam os valores de x,y,z da nuvem de pontos.
        vetx = np.append(vetx, float(data.points[i].x))
        vety = np.append(vety, float(data.points[i].y))
        vetz = np.append(vetz, float(data.points[i].z))

        #Distancia em 3d
        sum_root_3d = sqrt((vetx[i]**2 + vety[i]**2 + vetz[i]**2))



        # Controle de nivel dos 16 ponto em sequencia para varredura.
        if j >15:
            nivel = nivel +1
            j = 0


	#identifica pontos proximos.
        if  (sum_root_3d<5.00 or cont == 1):
            #Distancia em 2d	    
            sum_root_2d = sqrt((vetx[i]**2 + vetz[i]**2))

	

            controle_nivel = (nivel*16) -1


            if (sum_root_2d != 0.0 ):
	    	comp = np.append(comp, sum_root_2d)
		compz = np.append(compz, float(vety[i]))
            if (len(comp) == 2):
                #calcula angulo de inclinacao do objeto
                if (comp[0] == comp[1]):
                    dz = sqrt((compz[1]- compz[0])**2)
                    angle = degrees(atan2(dz,abs(comp[1])))	                              

               	else:
                    dz = sqrt((compz[1]- compz[0])**2)
                    dp = sqrt((comp[1]- comp[0])**2)
                    angle = degrees(atan2(dz,dp))
                if angle >30:
                    pares = pares + 1                        
                    #height = compz[1] #Altura do laser em relacao ao ponto
                    d2D =  comp[1] 
                    vetaz = np.append(vetaz,vety[i])
                comp = np.delete(comp,0)
                compz = np.delete(compz,0)
		
		if (d2D<1.00):
  		    print d2D
        
        # se o angulo for maior que um valor limite add a uma variavel contadora
        if (angle >= 30 and d2D<1.40 and pares >2):
                
            print "Amaerlo"
            print " Angulo ",angle,", altura ", compz,", distancia ", d2D, "\n" 
            print "Coordenadas x ", vetx[i],", y ",vety[i],", z ",vetz[i]
            print "\n"

            yellow = 1                          

	#se houver mais de p pontos com colisao risco de colisao que o obstaculo representa perigo.
	    if (angle >= 30 and d2D<1.00 and pares > 2):
                
                print "Vermelho"
	        print " Angulo ",angle,", altura ", compz,", distancia ", d2D, "\n" 
                print "Coordenadas x ", vetx[i],", y ",vety[i],", z ",vetz[i]
                print "\n"
                
                red = 2                      

                p_minx = np.append(p_minx,float(vetx[i])) 
                p_miny = np.append(p_miny,float(vety[i]))
                p_minz = np.append(p_minz,float(vetz[i]))
                #print np.amin(vetaz)                
                #print distancia
            pares = 0
	 






       #Setor onde ha perigo de colisao.
        if (12258<i<20480):
	     #Colisao Frontal.
             pontos_colisao = 1
        elif (8192<i<12258):
             #Colisao da quina frontal esquerda.
	     pontos_colisao = 2
        elif (20480<i<24576):
	     #Colisao da quina frontal direita.
             pontos_colisao = 3
        elif (4096<i<8192):
	     #Colisao da quina traseira esquerda.
             pontos_colisao = 4
        elif (24576<i<28672):
	     #Colisao da quina traseira direita.
             pontos_colisao = 5
        elif (28672<i<4096):
	     # Colisao traseira.
             pontos_colisao = 6
        else:
             pontos_colisao = 0
             check_collision = 0

       


        #Enquanto cont for igual a 1 ele continua varrendo meus 16 pontos.
        if (i<(controle_nivel)):
             cont = 1
        #Zera os valores pare calcular novo risco de colisao
        if (i == (controle_nivel)):
             cont = 0
             pontos = 0
             comp = []
             compz = []
             vetaz = []                
        #Incrimento de J para controle de nivel.
        j = j+1
	#Incremento de i para varrer os pontos.
        i = i+1






	#Fim do loop zero os pontos.
        if (i ==len(data.points)):
        
		if (yellow ==1 and red ==0):	
		        colisao = yellow
	                point_min.x = 0.0
	                point_min.y = 0.0
	                point_min.z = 0.0

	                #print "Amarelo "
	
        	if (red == 2):
                	colisao = red                
                	point_min.x = np.amin(p_minx)
                	point_min.y = np.amin(p_miny)
                	point_min.z = np.amin(p_minz)
	
                	#print "Vermehlo"

	        if (yellow == 0 and red == 0):
        	        colisao = green 
        	        point_min.x = 0.0
        	        point_min.y = 0.0
        	        point_min.z = 0.0
	
        	        #print "Verde"

        	i = 0
        	nivel = 0
        	vetx=[]
        	vety=[]
        	vetz=[]


def getdata():
	global colisao,old_colisao,proximity
	rospy.init_node("detect_colision",anonymous=True)
        #Altera aqui para o topico da nuvem de pontos
	rospy.Subscriber("/velodyne_points",PointCloud,callback)
	#Publica no topico colisao true se detectar colisao e false se nao.
	pub = rospy.Publisher("colisao",Int16,queue_size = 10)
	#Publica em qual setor do robo ocorreu a colisao.
	col = rospy.Publisher("pontos",Int32,queue_size = 10)
	#Publica menores valores de x e y quando o alerta e vermelho
	min_point = rospy.Publisher("/pontos_minimos",Vector3, queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
            #so altera valor no topico colisao caso ele mude
	        min_point.publish(point_min)
	    	col.publish(pontos_colisao)
	        if (old_colisao != colisao):
        	    old_colisao = colisao
	            pub.publish(old_colisao)                    
	   	rate.sleep()
if __name__== '__main__':
	try:
		getdata()
	except rospy.ROSInterruptException:
		pass
#!/usr/bin/env python
