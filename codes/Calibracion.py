import numpy as np
import cv2

from skimage import data, color
from skimage.transform import hough_circle, hough_circle_peaks
from skimage.feature import canny
from skimage.draw import circle_perimeter
from skimage.util import img_as_ubyte
from robodk.robolink import *
from robodk.robomath import *
import math
import time
from sklearn.linear_model import LinearRegression
import paho.mqtt.client as mqtt
import random

topico_general = "MeteoroRacers/MR2007B/#"
topico_sub  = "MeteoroRacers/MR2007B/Activacion_UR5"
topico_pub_acticar_herramienta = "MeteoroRacers/MR2007B/ActivacionHerramienta"
topico_pub_general = "MeteoroRacers/MR2007B/General"
topico_pub_torques = "MeteoroRacers/MR2007B/Torques"
frameWidth = 640
id = None
frameHeight = 480
cap = cv2.VideoCapture(2) 
cap.set(3, frameWidth)
cap.set(4, frameHeight)

RDK = Robolink()                        # connect to the RoboDK API
robot  = RDK.Item('', ITEM_TYPE_ROBOT)  # Retrieve a robot available in RoboDK
#target  = RDK.Item('Target 1')         # Retrieve a target (example)
cushion = RDK.Item('CusAss', ITEM_TYPE_OBJECT)
cushionInitialPose = TxyzRxyz_2_Pose([330.0, 190.00000000000006, 0.0, -0.0, 0.0, -0.0])
cushion.setPose(cushionInitialPose)


robot.setJoints([-180,-90,90,0,0,0])   

pose_home = robot.Pose()   
robot.MoveJ(pose_home)  


regression_RoboDK_X = LinearRegression()
regression_RoboDK_Y = LinearRegression()
regression_UR_X = LinearRegression()
regression_UR_Y = LinearRegression()
regression_UR_X2  = LinearRegression()
regression_UR_Y2 = LinearRegression()


def corregir_ojo_de_pez(image):
    height, width = image.shape[:2]

    # Parámetros de la cámara y de la distorsión
    focal_length = width
    center_x = width / 2
    center_y = height / 2
    camera_matrix = np.array([[focal_length, 0, center_x],
                              [0, focal_length, center_y],
                              [0, 0, 1]], dtype=np.float32)
    distortion_coefficients = np.zeros((4, 1), dtype=np.float32)

    # Realizar la corrección de ojo de pez
    undistorted_image = cv2.undistort(image, camera_matrix, distortion_coefficients)

    return undistorted_image

def empty(a):
    pass


def calibrar():
    global coordenadasCam
    with open("/home/alejandro/Documentos/Codigos/Python/Tec/SextoSemestre/AutomatizaciondeSistemasdeManufactura/RoboDKUR10CAM/PruebasFinales/HSVCountours.txt") as archivo:
            valores = archivo.readline()
            valores = valores.split(",")
            [h_min, h_max, s_min, s_max, v_min, v_max] = [int(valor) for valor in valores]

    cv2.namedWindow("HSV")
    cv2.resizeWindow("HSV", 640, 640)
    cv2.createTrackbar("HUE Min", "HSV", h_min, 179, empty)
    cv2.createTrackbar("HUE Max", "HSV", h_max, 179, empty)
    cv2.createTrackbar("SAT Min", "HSV", s_min, 255, empty)
    cv2.createTrackbar("SAT Max", "HSV", s_max, 255, empty)
    cv2.createTrackbar("VALUE Min", "HSV", v_min, 255, empty)
    cv2.createTrackbar("VALUE Max", "HSV", v_max, 255, empty)

    # Load picture and detect edges
    while True:
        _, image = cap.read()
        
        image = cv2.resize(image, [frameWidth, frameHeight], interpolation = cv2.INTER_AREA)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        #image = corregir_ojo_de_pez(image)

        h_min = cv2.getTrackbarPos("HUE Min", "HSV")
        h_max = cv2.getTrackbarPos("HUE Max", "HSV")
        s_min = cv2.getTrackbarPos("SAT Min", "HSV")
        s_max = cv2.getTrackbarPos("SAT Max", "HSV")
        v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
        v_max = cv2.getTrackbarPos("VALUE Max", "HSV")


        imgHsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(imgHsv, lower, upper)
        cv2.imshow("Mask_O", mask)

        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.imshow("Mask_BGR", mask)
        mask_gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        
        #image = img_as_ubyte(data.coins()[160:230, 70:270])
        edges = canny(mask_gray, sigma=0.2, low_threshold=100, high_threshold=240)
        
    

        # Detect two radii
        hough_radii = np.arange(3, 8, 1)
     
        hough_res = hough_circle(edges, hough_radii)

        # Select the most prominent 3 circles
        accums, cx, cy, radii = hough_circle_peaks(hough_res, hough_radii,
                                                total_num_peaks=10, min_ydistance=10, min_xdistance = 10)

        # Draw them

        image = color.gray2rgb(gray)
        for center_y, center_x, radius in zip(cy, cx, radii):
            
            circy, circx = circle_perimeter(center_y, center_x, radius,
                                            shape=image.shape)
            image[circy, circx] = (220, 20, 20)


        cv2.imshow("detected circles", image)
        if cv2.waitKey(30) >= 0 :
            break
    cv2.destroyAllWindows()
    cv2.imshow("detected circles", image)
    for center_y, center_x, radius, i in zip(cy, cx, radii, range(10)):
        coordenadasCam[i] = [center_x, center_y]

    

    coordenadasCam = coordenadasCam[coordenadasCam[:, 0].argsort()]
    coordenadasCamDerecha = coordenadasCam[:5]
    coordenadasCamIzquierda  = coordenadasCam[5:]
    coordenadasCam[:5] =  coordenadasCamDerecha[coordenadasCamDerecha[:, 1].argsort()]
    coordenadasCam[5:] = coordenadasCamIzquierda[coordenadasCamIzquierda[:, 1].argsort()]

    print("CoordenadasCam1")
    print(coordenadasCam)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    ArchivoCalibracion = open("/home/alejandro/Documentos/Codigos/Python/Tec/SextoSemestre/AutomatizaciondeSistemasdeManufactura/RoboDKUR10CAM/PruebasFinales/HSVCountours.txt","w")
    data = str(h_min)+","+str(h_max)+","+str(s_min)+","+str(s_max)+","+str(v_min)+","+str(v_max)
    ArchivoCalibracion.write(data)
    ArchivoCalibracion.close()



def calibrarPos():

    
    global coordenadasCam
    coordenadasRoboDk = np.array([[619.933, 24.4], [619.933, 89.6], [619.933,233.6], [619.933, 328.1], [619.933, 355.1],
                        [39.933, 24.4],[39.933, 89.6], [39.933, 233.6], [39.933, 328.1], [39.933, 355.1]])
    
    coordenadasURRegresion = np.array([[452.858, 393.432], [454.700, 421.278], [457.655, 516.826], [460.939, 664.071], [460.237, 731.471], 
                             [-104.317, 409.429], [-104.653, 436.179], [-105.899, 529.465], [-109.874, 673.824], [-112.307, 739.502]])
    

    posRoboDk_x  = coordenadasRoboDk[:, 0]
    posRoboDk_y  = coordenadasRoboDk[:, 1]

    posUR_x = coordenadasURRegresion[:, 0]
    posUR_y = coordenadasURRegresion[:, 1]

    camX = coordenadasCam[:, 0].reshape(-1, 1)
    camY = coordenadasCam[:, 1].reshape(-1, 1)
  
    regression_RoboDK_X.fit(camX[:5], posRoboDk_x[:5])
    regression_RoboDK_Y.fit(camY[:5], posRoboDk_y[:5])

    regression_UR_X.fit(camX[:5], posUR_x[:5])
    regression_UR_Y.fit(camY[:5], posUR_y[:5])
    regression_UR_X2.fit(camX[5:], posUR_x[5:])
    regression_UR_Y2.fit(camY[5:], posUR_y[5:])

    ArchivoCalibracion = open("/home/alejandro/Documentos/Codigos/Python/Tec/SextoSemestre/AutomatizaciondeSistemasdeManufactura/RoboDKUR10CAM/PruebasFinales/Coeficientes.txt","w")
    data = str(regression_UR_X.coef_[0])+","+str(regression_UR_X.intercept_)+","+str(regression_UR_Y.coef_[0])+","+str(regression_UR_Y.intercept_)+","+str( regression_UR_X2.coef_[0])+","+str( regression_UR_X2.intercept_)+","+str( regression_UR_Y2.coef_[0])+","+str( regression_UR_Y2.intercept_)+","+str( regression_RoboDK_X.coef_[0])+","+str( regression_RoboDK_X.intercept_)+","+str( regression_RoboDK_Y.coef_[0])+","+str( regression_RoboDK_Y.intercept_)
    ArchivoCalibracion.write(data)
    ArchivoCalibracion.close()
   
    
def leerQR(image):
    ret_qr = None

    mirrored_image = cv2.flip(image, 1)
    #print("a")
    detect = cv2.QRCodeDetector()

    retval, decoded_info, points, straight_qrcode = detect.detectAndDecodeMulti(image)
    #print(retval)
    print(f"QRCode data:\n{decoded_info}")
    if decoded_info is not None:
        print(f"QRCode data:\n{decoded_info}")
        
        # display the image with lines4
        return decoded_info
    else:
        return None
    
        



def detectarCirculos():
    global coordenadasCam
    global id
    tornillosDetecados = False
    codigoNoDetectado = True
    with open("/home/alejandro/Documentos/Codigos/Python/Tec/SextoSemestre/AutomatizaciondeSistemasdeManufactura/RoboDKUR10CAM/PruebasFinales/HSVCountours.txt") as archivo:
        valores = archivo.readline()
        valores = valores.split(",")
        [h_min, h_max, s_min, s_max, v_min, v_max] = [int(valor) for valor in valores]
    for i in range(50):
            _, image = cap.read()
            time.sleep(10/1000)
    while(tornillosDetecados == False):
        _, image = cap.read()

        image = cv2.resize(image, [frameWidth, frameHeight], interpolation = cv2.INTER_AREA)

        if codigoNoDetectado:
            id = leerQR(image)
            if id is not None:
                codigoNoDetectado = False
            
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        #image = corregir_ojo_de_pez(image)



        imgHsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(imgHsv, lower, upper)
        cv2.imshow("Mask_O", mask)

        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.imshow("Mask_BGR", mask)
        mask_gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        
        #image = img_as_ubyte(data.coins()[160:230, 70:270])
        edges = canny(mask_gray, sigma=0.2, low_threshold=100, high_threshold=240)
        
    

        # Detect two radii
        hough_radii = np.arange(3, 4, 1)
        
        hough_res = hough_circle(edges, hough_radii)

        # Select the most prominent 3 circles
        accums, cx, cy, radii = hough_circle_peaks(hough_res, hough_radii,
                                                total_num_peaks=10, min_ydistance=10, min_xdistance = 10)

        # Draw them

        image = color.gray2rgb(gray)
        for center_y, center_x, radius in zip(cy, cx, radii):
            
            circy, circx = circle_perimeter(center_y, center_x, radius,
                                            shape=image.shape)
            image[circy, circx] = (220, 20, 20)
            cv2.putText(image, str(center_x)+", " + str(center_y), (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        
        tornillosDetecados = True

    cv2.destroyAllWindows()
    cv2.imshow("detected circles", image)
    # for center_y, center_x, radius, i in zip(cy, cx, radii, range(10)):
    #     coordenadasCam[i] = [center_x, center_y]
        
    # coordenadasCam = coordenadasCam[coordenadasCam[:, 0].argsort()]
    # coordenadasCamDerecha = coordenadasCam[:5]
    # coordenadasCamIzquierda  = coordenadasCam[5:]
    # coordenadasCam[:5] =  coordenadasCamDerecha[coordenadasCamDerecha[:, 1].argsort()]
    # coordenadasCam[5:] = coordenadasCamIzquierda[coordenadasCamIzquierda[:, 1].argsort()]
   
    print("Coordenadas Cam:")
    print(coordenadasCam)
    publish(client, topico_pub_general, "Circulos Detectados")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def definirTargetPoints():
    
    global coordenadasCam
    global coordenadasUR
    distances = np.array([25, 65.2, 144, 94.5, 27, 25])
    #580
    with open("/home/alejandro/Documentos/Codigos/Python/Tec/SextoSemestre/AutomatizaciondeSistemasdeManufactura/RoboDKUR10CAM/PruebasFinales/Coeficientes.txt") as archivo:
        valores = archivo.readline()
        valores = valores.split(",")
        [m_x, b_x, m_y, b_y, m_x2, b_x2, m_y2, b_y2, _, _, _, _] = [float(valor) for valor in valores]


    print("Coeficinetes de la regresion de UR")
    print(m_x)
    print(b_x)
    print(m_y)
    print(b_y)

    delta_x = coordenadasCam[1][0] - coordenadasCam[0][0]
    delta_y =  coordenadasCam[1][1] - coordenadasCam[0][1]
    angulo = math.atan2(delta_y, delta_x)-math.pi/2
    print(angulo)

    for i, point in enumerate(coordenadasCam[:5]): #Recorre la matriz para cada círculo detectado.
        coordenadasUR[i][0] = point[0]*m_x+b_x
        coordenadasUR[i][1] = point[1]*m_y+b_y



    for i, point in enumerate(coordenadasCam[5:]): #Recorre la matriz para cada círculo detectado.
        coordenadasUR[i+5][0] = point[0]*m_x2+b_x2
        coordenadasUR[i+5][1] = point[1]*m_y2+b_y2


    print("Coordenadas UR:")
    print(coordenadasUR)
    #coordenadasUR = np.array([[460.237, 731.471], [460.939, 664.071], [457.655, 516.826], [454.700, 421.278], [452.858, 393.432], 
    #                         [-112.307, 739.502], [-109.874, 673.824], [-105.899, 529.465], [-104.653, 436.179], [-104.317, 409.429]])
    
 


def moverUR():
    target_sobre_tornillo = np.zeros((10, 6))
    target_tornillo = np.zeros((10, 6))
    z_sobremesa = 267
    z_mesa = 237.417

    #En radianes
    u_default = 0.049
    v_default = 2.187
    w_default = 2.202




    for i, punto in enumerate(coordenadasUR):
        target_sobre_tornillo[i] = [punto[0], punto[1], z_sobremesa, u_default, v_default, w_default]
        target_tornillo[i] = [punto[0], punto[1], z_mesa, u_default, v_default, w_default]
        
    # print("Target")
    # print(target_sobre_tornillo)


    robot.MoveJ(pose_home) 
    global id

    for i in range(10):
        robot.setSpeed(-1, 50)
        robot.MoveJ(UR_2_Pose(target_sobre_tornillo[i])) 
        publish(client,topico_pub_acticar_herramienta, "True")
        robot.setSpeed(50,-1)
        robot.MoveL(UR_2_Pose(target_tornillo[i]))
        publish(client,topico_pub_acticar_herramienta, "False")
        robot.setSpeed(-1, 50)
        robot.MoveJ(UR_2_Pose(target_sobre_tornillo[i]))   
        if (i == 4):
            robot.setSpeed(-1, 50)
            robot.MoveJ(UR_2_Pose([147.845, 704.479, 248.337, 0.059, 2.207, 2.197])) 

    robot.MoveJ(UR_2_Pose([147.845, 704.479, 248.337, 0.059, 2.207, 2.197])) 
    
    robot.MoveJ(pose_home)

    torques = np.random.rand(10)
    publish(client, topico_pub_general,  "Movimiento Terminado")
    id = ''.join(random.choice('0123456789') for i in range(4))
    id = id.join(random.choice('ABCDEFGHIJK') for i in range(3))
    id = id.join(random.choice('0123456789') for i in range(3))
    publish(client, topico_pub_general,  "Movimiento Terminado")
    publish(client, topico_pub_torques,  str(id)+","+str(torques[0])+","+str(torques[1])+","+str(torques[2])+","+str(torques[3])+","+str(torques[4])+","+str(torques[5])+","+str(torques[6])+","+str(torques[7])+","+str(torques[8])+","+str(torques[9]))



def distancia_punto(punto, punto2):
    return math.sqrt((punto[0]-punto2[0])**2 + (punto[1]-punto2[0])**2)


def actualizarCushion():
    global coordenadasCam
    # print("LAs coordenadas orignales son: ")
    # print(coordenadasCam)
    coordenadasCamCopy = np.zeros(((10, 2)))

    with open("/home/alejandro/Documentos/Codigos/Python/Tec/SextoSemestre/AutomatizaciondeSistemasdeManufactura/RoboDKUR10CAM/PruebasFinales/Coeficientes.txt") as archivo:
        valores = archivo.readline()
        valores = valores.split(",")
        [_, _, _, _, _, _, _, _, m_x, b_x, m_y, b_y] = [float(valor) for valor in valores]

    for i, punto in enumerate(coordenadasCam):
        coordenadasCamCopy[i][0] = coordenadasCam[i][0]
        coordenadasCamCopy[i][1] = coordenadasCam[i][1]


    distanciaMinima = float('inf')
    puntoMasCercano = None
    

    for punto in (coordenadasCamCopy): #Recorre la matriz para cada círculo detectado.
        
        distancia = distancia_punto(punto, [0, 480])
        if distancia < distanciaMinima:
            distanciaMinima = distancia
            puntoMasCercano = punto
    

    distanciaMinima = float('inf')
    for punto in coordenadasCamCopy: #Recorre la matriz para cada círculo detectado.
        if (not(np.array_equal(punto, puntoMasCercano))):
            distancia = distancia_punto(punto, [0, 480])
            if distancia < distanciaMinima:
                distanciaMinima = distancia
                puntoMasCercano2 = punto


    delta_x = puntoMasCercano2[0] - puntoMasCercano[0]
    delta_y =  puntoMasCercano2[1] - puntoMasCercano[1]
    angulo = math.atan2(delta_y, delta_x)-math.pi/2
    

    #TRansformar los puntos a coordenadas en el SR del cushion
 

    print("Coeficinetes de la regresion de Robodk")
    print(m_x)
    print(b_x)
    print(m_y)
    print(b_y)

    coordenadasCushion[0][0] = puntoMasCercano[0]*m_x+b_x
    coordenadasCushion[0][1] = puntoMasCercano[1]*m_y+b_y

    CentroCushion = [coordenadasCushion[0][0]-289.933,coordenadasCushion[0][1]+114.806]

    

    # for i, point in enumerate(coordenadasCam): #Recorre la matriz para cada círculo detectado.
    #     coordenadasCushion[i][0] = point[0]*m_x+b_x
    #     coordenadasCushion[i][1] = point[1]*m_y+b_y

    
    print("Centro Cushion:")
    print(CentroCushion)
    print("Angulo:")
    print(np.rad2deg(angulo))
    

    cushionNuevaPosicion = TxyzRxyz_2_Pose([CentroCushion[0],CentroCushion[1], 0.0, -0.0, 0.0, angulo])

    cushion.setPose(cushionNuevaPosicion)



def on_connect(client, userdata, flags, rc):
    print("Se conecto con mqtt " + str(rc))
    client.subscribe(topico_general)


def on_message(client, userdata, msg):
    global empezarRutina
    print("received")
    if msg.topic == topico_sub:
        print(msg.payload)
        if (msg.payload.decode("utf-8") == 'True'):
            print("received23")
            empezarRutina = True
        
    print(msg.topic + " " + (msg.payload).decode("utf-8"))


def publish(client, topico_pub, msg):
    msg_count = 1
    while True:
        
        
        result = client.publish(topico_pub, msg)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            print(f"Send `{msg}` to topic `{topico_pub}`")
        else:
            print(f"Failed to send message to topic {topico_pub}")
        break

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("broker.hivemq.com", 1883, 60)



coordenadasCam = np.zeros((10, 2))
coordenadasUR = np.zeros((10, 2))
coordenadasCushion = np.zeros((10, 2))
     


empezarRutina = False
while True:

    
    if (empezarRutina):
        calibrar()
        calibrarPos()
        detectarCirculos()
        actualizarCushion()
        definirTargetPoints()
        moverUR()
        empezarRutina  = False
    client.loop_start()
