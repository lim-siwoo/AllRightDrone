# AllRightDrone
Drone Control By Pose Estimation Using Media Pipe

## Introduction
*Name* : AllRight Drone
*Platform* : DJI Telllo
*Tech Stack* : Python OpenCV, DJITelloPy
*Purpose* : Purpose Control drone with human pose detection

## Project Features
* Estimate human pose
* Command by pose gesture
* No controller required
* Easy to control

## DJI Tello
*Tello DJI* is and programmable drone perfect tor education
*DJITelloPy* is a Python libraray using the Tello EDU SDK
  - Provides various functions to control drones

## Tech Stack
 *OpenCV* is an open-source library that includes serveral hundreds of computer vison algorithms.
  - Using Python
  - For Human pose estimate
 *Pose Estimation* from real-time feed plays a crucial role in various fields such ad full-body gesture control
  - Detect human pose
  - In real-time video
 *Media Pipe Pose* is a framework for high-fidelity body pose tracking.
  - Takes input from RGB video frams
  - Infers 33 3D landmarks on the whole human
  - This method outperforms other methods and achieve very good results in real-time
  <img src="https://user-images.githubusercontent.com/15250755/200211590-875a9a08-b788-4b57-a9de-90f6f7e2620d.png" width="40%" height="30%" title="그림1" alt="media pipe"></img>
  <img src="https://user-images.githubusercontent.com/15250755/200211596-f93dc805-b871-4d08-ae78-db9de1f4f131.png" width="40%" height="30%" title="그림2" alt="media pipe2"></img>
 
 ## Follow Up
 + Using the human nose as a reference point
 + Change direction while tracking location based on the center of the screen
 + When the person disappearss from the screen, it rotates based on their last position
 + if multiple people come in, kepp track of the first detected person drone tracked
 <img src="https://user-images.githubusercontent.com/15250755/200211598-f54621f5-0390-4ee1-8d49-1ff7c4a45ee7.png" width="40%" height="30%" title="그림3" alt="follow up"></img>
 ### Follow up Algorithm
  - Error rate calculation
  - Move by a constance speed margin
  - If person is lost, move based on pervious_error
 ## Detectable Pose types
  * Move Left   
  <img src="https://user-images.githubusercontent.com/15250755/200212857-eb5f3ee2-d39a-4c22-a3b5-e4ea7ac5964f.png" width="20%" height="15%" title="그림4" alt="Move Left"></img>
  * Move Right   
  <img src="https://user-images.githubusercontent.com/15250755/200212862-7ab676c7-6a53-457a-aa67-2e750b887419.png" width="20%" height="15%" title="그림5" alt="Move Right"></img>
  * Take a Picture   
  <img src="https://user-images.githubusercontent.com/15250755/200212863-01ecc92d-cea1-4809-9a95-4608aeff54f9.png" width="20%" height="25%" title="그림6" alt="Take a Picture"></img>
  * Landing   
  <img src="https://user-images.githubusercontent.com/15250755/200212865-cf19b89a-9908-4ff9-add5-f39823360c6b.png" width="20%" height="15%" title="그림7" alt="Landing"></img>

## Pose Estimation
  * Implement angle calculation of body joints to estimation pose
  * Judging a pose based on the calculated angle
  * Multiple joint were cross-validated to prevent malfunctions.   
  <img src="https://user-images.githubusercontent.com/15250755/200213808-8f61183b-549a-4e60-a4c5-2ea31b5f4cc2.png" width="20%" height="15%" title="그림8" alt="Code1"></img>
  <img src="https://user-images.githubusercontent.com/15250755/200213812-398d5206-d257-4f91-bbe5-9afc22b4350b.png" width="20%" height="15%" title="그림9" alt="Code2"></img>
  
## YouTube Demo

  [![Demo1](http://img.youtube.com/vi/jJgFWLUho00/0.jpg)](https://www.youtube.com/watch?v=jJgFWLUho00) 
  
  [![Demo2](http://img.youtube.com/vi/lAnxkaXryik/0.jpg)](https://www.youtube.com/watch?v=lAnxkaXryik) 
  
## Team Role
*임시우* : OpenCV Development
*김범기* : Drone Control Development



    
