# UR-Vision
<p align="justify">
This repository contains the work I made for the ADIENT company. They wanted to automate the process of screwing a certain piece, so using a camera as an Eye In the Sky and a UR10 robotic arm, I made a vision system that detected the position of the screws, and then using a Coordinate Transformation, I got the position of the Screws in the robot Reference Frame. With this, and using the software of RoboDK, I was able to control the robot's movements, have a cyber-physical system of this part of the process in RoboDK, and send information about the piece to a dashboard in the cloud.
</p>

You can find more information about the algorithms I made in the [codes](https://github.com/alejandro3141592/UR-Vision/tree/1280377e6da75b53db022f97a132acd944cce6f1/codes) folder. 


In the following image you can see some circles drawn in red, these circles are the ones that the algorithm detected as screws.
<p align="center">
<img src="https://github.com/alejandro3141592/UR-Vision/assets/132953325/cd89a47e-f92e-4749-9277-6182d7c18f82"/>
</p>


In the following gif, you can see how the robot moves to every screw in the piece.
<p align="center">
<img src="https://github.com/alejandro3141592/UR-Vision/assets/132953325/4900cd7d-b21a-4c40-b689-03997822562b"/>
</p>
