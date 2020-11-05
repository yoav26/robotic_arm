# Robotic Arm for NDT
![robotic_arm_dev](https://user-images.githubusercontent.com/59603146/97998404-b885ce00-1df2-11eb-9ac4-251dfda7d4bc.jpeg)

## Abstract

Today the use of composites materials is wider than ever, we are witnessing a significant change in traditional production methods and replacing or combining steel with light and strong materials, this presents many challenges with the main one being testing the quality of the structure manufactured, both after production and periodic maintenance tests. The type of test used is Non-Destructive-Test (NDT) This type of testing is becoming common and not for nothing, there are many benefits to applying this practice in many industries, the main advantage is that the test done on the structure itself that been used, and it’s not destructive. Today the most common types of NDT are of pipes welds and large tanks in the petrochemical industry, where automation is used quite a bit because of the need to go into a tank and scan its inside without cutting it, or to test a remote pipe section. In the aerospace industry structures built of composite materials such as wings are characterized by multiple supporting spaces and branches, today the test is done manually, performing this test by a technician poses difficulties because it is hard labor done in an unergonomic environment therefore there is great danger of injuries and work diseases. In addition, manual testing puts noise into results due to operator instability, and there are inaccessible places.
 
 For the reasons stated, it can be understood that automation of these operations is a significant market need today. Building a robotic arm capable of performing the test efficiently will improve the quality of the test, allow access to places with limited accessibility for technicians, save manpower costs and enable maintenance tests to be performed more frequently. Moreover, additional benefits will be reflected, a higher level of safety and accurate designing with the possibility of minimizing safety factors taken due to lack of maintenance tests.
 
In this project we want to build an automated system for non-destructive-testing of wings, a branched structure with very limited accessibility, for this purpose it was decided to develop a hyper-redundant-articulated-robot (Snake Robot) whose job is to successfully navigate within the branched structure the end-effector i.e. the robotic arm, to the desired scanning area. The project is divided into two separate projects - Snake Robot, whose job is to reach the required scanning area and an end unit whose job is to perform the actual scanning on the desired surface. This project focuses on the development of the end-effector whose job is to perform the actual scanning using an ultrasonic tracker.

### Video
[![Watch the video](https://img.youtube.com/vi/91QBKouiPIk/maxresdefault.jpg)](https://youtu.be/91QBKouiPIk)

### --version Python 2.7.12
