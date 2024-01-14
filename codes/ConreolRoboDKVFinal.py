# -*- coding: utf-8 -*-

from robolink import *
from robodk import *
import cv2
import numpy as np
from robodk.robolink import *
from robodk.robomath import *
import imutils
import math

CAM_NAME = 'Camera 1'
ROBOT_NAME1 = 'UR5 A'
CAM_PARAMS = 'SIZE=640x480' # For more options, see https://robodk.com/doc/en/PythonAPI/robodk.html#robodk.robolink.Robolink.Cam2D_Add
WINDOW_NAME = 'My camera'
NUM_TARGETS1 = 21
rdk_ip = 'localhost'
DELTA_Y1 = -1000


RDK = Robolink(robodk_ip=rdk_ip, port=20500)


robot = RDK.Item(ROBOT_NAME1, ITEM_TYPE_ROBOT)
robotMovil = RDK.Item("Omron LD-60", ITEM_TYPE_ROBOT)

cam_item = RDK.Item("Camera 1", ITEM_TYPE_CAMERA)
print(cam_item.Valid())
cam_item.setParam(CAM_PARAMS)


reference_preatornillado = RDK.Item("Frame 3", ITEM_TYPE_FRAME)
UR_base = RDK.Item("UR5 Base1", ITEM_TYPE_FRAME)
Omron_base = RDK.Item("Omron LD-60 Base", ITEM_TYPE_FRAME)


def MakePoints(xStart, xEnd, numPoints):

    """Generates a list of points"""
    if len(xStart) != 3 or len(xEnd) != 3:
        raise Exception("Start and end point must be 3-dimensional vectors")
    if numPoints < 2:
        raise Exception("At least two points are required")

    # Starting Points
    pt_list = []
    x = xStart[0]
    y = xStart[1]
    z = xStart[2]

    # How much we add/subtract between each interpolated point
    x_steps = (xEnd[0] - xStart[0]) / (numPoints - 1)
    y_steps = (xEnd[1] - xStart[1]) / (numPoints - 1)
    z_steps = (xEnd[2] - xStart[2]) / (numPoints - 1)

    # Incrementally add to each point until the end point is reached
    for i in range(numPoints):
        point_i = [x, y, z]  # create a point
        #append the point to the list
        pt_list.append(point_i)
        x = x + x_steps
        y = y + y_steps
        z = z + z_steps
    return pt_list



def get_wrist_simulated_image():

  img = None
  import tempfile
  with tempfile.TemporaryDirectory(prefix='rdk_') as tf:
     t_dir = tf + '/temp.png'
     if RDK.Cam2D_Snapshot(t_dir):
        img = cv2.imread(t_dir)
  return img
  


def detect_circles(image):

  image_gris = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #Convierte la imagen a escala de grises
  blurred = cv2.GaussianBlur(image_gris, (5, 5), 0) #Aplica un filtro gaussiano para suavizar la imagen
  circulos = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT,1,40,
                            param1=90,param2=22,minRadius=0,maxRadius=100) #Hace la detecciónd de los círculos y regreasa una matriz con los parámetros de cada círculos
  
  if (circulos is not None): #Si detectó algun círculo
    circulos = np.uint16(np.around(circulos))
    for circulo in circulos [0,:]: #Recorre la matriz para cada círculo detectado.
          
        cv2.circle(image, (circulo[0],circulo[1]), circulo[2], (0, 0, 255), 2) #Dibuja un círculo con los parámetros indicados, en la imagen original

   
  return image
  

  

def read_qr_code(frame):
    ret_qr = None

    mirrored_image = cv2.flip(frame, 1)
    print("a")
    detect = cv2.QRCodeDetector()
    retval, decoded_info, points, straight_qrcode = detect.detectAndDecodeMulti(mirrored_image)
    print(retval)
    print(decoded_info)
        
            
    return decoded_info

    



def trasladar(direccion: int, target_p):
    
    new_poseArm = Omron_base.Pose()
    new_p= Pose_2_UR(new_poseArm)[direccion]

    error = target_p - new_p

    while (error!=0):
        
        old_poseArm = Omron_base.Pose()
        xyzuvwArm = Pose_2_UR(old_poseArm)
        print(xyzuvwArm)

        old_poseMovil= Omron_base.Pose()
        xyzuvwMovil = Pose_2_UR(old_poseMovil)


        if(abs(error)>50):
            correccion = 30*(error/abs(error))
        elif(abs(error)>10):
            correccion = 10*(error/abs(error))
        else:
            correccion = error

        xyzuvw2Arm = xyzuvwArm
        xyzuvw2Arm[direccion] = xyzuvw2Arm[direccion]+correccion
        new_poseArm = UR_2_Pose(xyzuvw2Arm)  

        xyzuvw2Movil= xyzuvwMovil
        xyzuvw2Movil[direccion] = xyzuvw2Movil[direccion]+correccion
        new_poseMovil = UR_2_Pose(xyzuvw2Movil)  

    
        #UR_base.setPose(new_poseArm)
        Omron_base.setPose(new_poseMovil)

        new_p= Pose_2_UR(new_poseArm)[direccion]
        error = target_p - new_p


def trasladar2(target_x, target_y):
    
    correccion = np.zeros(2)
    error = np.zeros(2)
    new_p = np.zeros(2)
    new_poseMovil = Omron_base.Pose()
    new_p[0]= Pose_2_UR(new_poseMovil)[0]
    new_p[1]= Pose_2_UR(new_poseMovil)[1]
    
    target_p = [target_x, target_y]
    error[0] = target_p[0] - new_p[0]
    error[1] = target_p[1] - new_p[1]

    while (error[0]!=0 or error[1]!=0):
        
        old_poseArm = Omron_base.Pose()
        xyzuvwArm = Pose_2_UR(old_poseArm)
        print(xyzuvwArm)

        old_poseMovil= Omron_base.Pose()
        xyzuvwMovil = Pose_2_UR(old_poseMovil)


        if(abs(error[0])>50):
            correccion[0] = 30*(error[0]/abs(error[0]))
        elif(abs(error[0])>10):
            correccion[0] = 10*(error[0]/abs(error[0]))
        else:
            correccion[0] = error[1]
        if(abs(error[1])>50):
            correccion[1] = 30*(error[1]/abs(error[1]))
        elif(abs(error[1])>10):
            correccion[1] = 10*(error[1]/abs(error[1]))
        else:
            correccion[1] = error[1]

        xyzuvw2Arm = xyzuvwArm
        xyzuvw2Arm[0] = xyzuvw2Arm[0]+correccion[0]
        xyzuvw2Arm[1] = xyzuvw2Arm[1]+correccion[1]
        new_poseArm = UR_2_Pose(xyzuvw2Arm)  

        xyzuvw2Movil= xyzuvwMovil
        xyzuvw2Movil[0] = xyzuvw2Movil[0]+correccion[0]
        xyzuvw2Movil[1] = xyzuvw2Movil[1]+correccion[1]
        new_poseMovil = UR_2_Pose(xyzuvw2Movil)  

    
        #UR_base.setPose(new_poseArm)
        Omron_base.setPose(new_poseMovil)

        new_p[0]= Pose_2_UR(new_poseMovil)[0]
        new_p[1]= Pose_2_UR(new_poseMovil)[1]
        error[0] = target_p[0] - new_p[0]
        error[1] = target_p[1] - new_p[1]



def rotar(direccion: int, target_p):
    
    new_poseArm = Omron_base.Pose()
    new_p= Pose_2_UR(new_poseArm)[direccion+3]
    error = target_p - new_p

    old_poseMovil= Omron_base.Pose()
    xyzuvwMovil = Pose_2_UR(old_poseMovil)

    while (abs(error)>pi/300):
        print(error)
        old_poseArm = Omron_base.Pose()
        xyzuvwArm = Pose_2_UR(old_poseArm)
        print(xyzuvwArm)

        if(abs(error)>pi/20):
            correccion = pi/45*(error/abs(error))
        elif(abs(error)>pi/60):
            correccion = pi/70*(error/abs(error))
        else:
            correccion = error

        xyzuvw2Arm = xyzuvwArm
        xyzuvw2Arm[direccion+3] = xyzuvw2Arm[direccion+3]+correccion
        new_poseArm = UR_2_Pose(xyzuvw2Arm)  


        xyzuvw2Movil= xyzuvwMovil
        xyzuvw2Movil[direccion+3] = xyzuvw2Movil[direccion+3]+correccion
        new_poseMovil = UR_2_Pose(xyzuvw2Movil) 
    
        #UR_base.setPose(new_poseArm)
        Omron_base.setPose(new_poseMovil)

        new_p= Pose_2_UR(new_poseArm)[direccion+3]
        error = target_p - new_p



if __name__ == "__main__":

    print("a")
    RDK.Render(True)
    print("b")


    # Abort if the user hits Cancel
    if not robot.Valid():
        quit()

    # Retrieve the robot reference frame
    #reference = robot.Parent()
    
    # Use the robot base frame as the active reference
    
    robot.setPoseFrame(reference_preatornillado)

    first_pose = UR_base.Pose()
    first = Pose_2_UR(first_pose)
    print(first)
    targets = RDK.ItemList(ITEM_TYPE_TARGET)

    #Leer QR    
    cam_item.setParam('Open', 1)
    robot.MoveJ(targets[-1])
    
    frame = get_wrist_simulated_image()

    value = read_qr_code(frame) #Detecta cuadrados
    

    cv2.putText(frame,value[0], (100,400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1, 2)
    cv2.imshow(WINDOW_NAME, frame) #Lo muestra en una ventana
    cv2.waitKey(0) #Comando que evita que las ventanas se cierren al instante, espera a que se presione la tecla 0
    cv2.destroyAllWindows() #Destrue las ventanas luego de que se oprima la tecla
    
    #Leer primer tornillo
    robot.MoveJ(targets[-2])
    frame = get_wrist_simulated_image()
    frame = detect_circles(frame)
    cv2.imshow(WINDOW_NAME, frame) #Lo muestra en una ventana
    cv2.waitKey(0) #Comando que evita que las ventanas se cierren al instante, espera a que se presione la tecla 0
    cv2.destroyAllWindows() #Destrue las ventanas luego de que se oprima la tecla


    for i in range(NUM_TARGETS1):

            # update the reference target with the desired XYZ coordinates
                robot.MoveJ(targets[i])
                #frame = get_wrist_simulated_image()

                #value = read_qr_code(frame) #Detecta cuadrados
                #if not(value == None):
                 #   print(value)

                #cv2.imshow(WINDOW_NAME, frame) #Lo muestra en una ventana
                # key = cv2.waitKey(1)
                # if key == 27:
                #     break  # User pressed ESC, exit
                # if not cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE):
                #     print("Fin")
                #     break  # User killed the main window, exit
    robot.MoveJ(targets[0])
    #dirección = 0 para x, 1 pra y,2 paea z
    trasladar(1, 300)
    rotar(2, math.pi/4)

    # [     0.000000,    -1.000000,     0.000000,  1756.531000 ;
    #   1.000000,     0.000000,     0.000000,  1262.661000 ;
    #   0.000000,     0.000000,     1.000000,     0.000000 ;
    #   0.000000,     0.000000,     0.000000,     1.000000 ];

    #estacionarse()
    trasladar2(1606.531, 150)
    rotar(2,0)

    trasladar(0, 2255.137)

    for i in range(NUM_TARGETS1+1):
        
            # update the reference target with the desired XYZ coordinates
                robot.MoveJ(targets[i+NUM_TARGETS1])
                #frame = get_wrist_simulated_image()

                #value = read_qr_code(frame) #Detecta cuadrados
                #if not(value == None):
                 #   print(value)

                #cv2.imshow(WINDOW_NAME, frame) #Lo muestra en una ventana
                # key = cv2.waitKey(1)
                # if key == 27:
                #     break  # User pressed ESC, exit
                # if not cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE):
                #     print("Fin")
                #     break  # User killed the main window, exit    

    trasladar(0, 1606.531)
    rotar(2, math.pi/4)

    trasladar2(1756.531, 300)
    rotar(2, math.pi/2)

 
    rotar(2, math.pi/2)
    trasladar(1, 1262.661)
    
#
    
    # new_pose = UR_base.Pose()
    # new_y = Pose_2_UR(first_pose)[1]
    # print(new_y)
    # print(first_y)
    # while (new_y-first_y>DELTA_Y1):
    #     old_poseArm = UR_base.Pose()
    #     xyzuvw = Pose_2_UR(old_poseArm)
    #     print(xyzuvw)
    #     x,y,z,u,v,w = xyzuvw                    # Use the KUKA representation (for example) and calculate a new pose based on the previous pose
    #     new_y = y-20
    #     xyzuvw2 = [x,new_y,z,u,v,w]
    #     new_pose = UR_2_Pose(xyzuvw2)  
    
    #     UR_base.setPose(new_pose)

    UR_base.setPose(first_pose)
    robot.MoveJ(targets[0])
    print(first)

    # Close the preview and the camera. Ensure you call cam_item.setParam('Open', 1) before reusing a camera!
    cv2.destroyAllWindows() #Cierra la ventana
    RDK.Cam2D_Close(cam_item) #Cierra la cámara
    

