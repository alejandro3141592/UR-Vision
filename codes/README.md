In this folder you will find two codes, the Calibracion.py file, contains a routine that helped me to change the Robot Reference Frame if for any reason was moved, and it also allows me to test different values for an HSV mask to choose the ones that give me better results in certain conditions of light.
On the other hand, the file ControlRoboDKFinal.py contains the main code that uses the camera and an HSV filter, allowing me to identify the screws, then, a transformation was made to these to obtain the coordinates of every screw in the robot reference frame, and then, sends the commands using the RoboDK software to the UR10.
![EstaciónAtornillado2](https://github.com/alejandro3141592/UR-Vision/assets/132953325/83f8ced3-18b9-4154-ab9b-d5f354dcc65d)
![Vídeo sin título ‐ Hecho con Clipchamp (3)](https://github.com/alejandro3141592/UR-Vision/assets/132953325/8561b525-4b30-4385-9568-ac1cfa41b130)