# Robotics-MotionPlanning-SampleBased

HW Guide: https://github.com/notsky23/Robotics-MotionPlanning-SampleBased/blob/master/hw3-2.pdf.<br><br>

## What is this practice about?<br>

This module is a practice with robot kinematics and motion planning.<br><br>

We will be using 2 well known Sampling-based motion planning algorithms:<br>
1. Probabilistic roadmaps (PRM)<br>
2. Rapidly-exploring random trees (RRT)<br><br>

## Results:<br>

Here are the results I got.<br>

The code is included in this repo.<br><br>

### M0 - Plotting of robot in workspace:<br>

<img src="https://user-images.githubusercontent.com/98131995/234774183-aa43c871-c027-4e08-88fc-be1bba319672.png" width=50% height=50%><br><br>
![image](https://user-images.githubusercontent.com/98131995/234774248-c49252f4-8ae4-4d06-8d06-47ec421bfb46.png)<br><br>

### M1 - Collision check:<br>

![Collision check](https://user-images.githubusercontent.com/98131995/234804794-a9d0423c-da91-469b-a42b-1b7e360637c8.gif)<br>
![image](https://user-images.githubusercontent.com/98131995/234805103-07ebc4bf-dc0b-4d7c-b578-de683460f071.png)<br><br>

### M2 - PRM (Probabilistic Roadmaps):<br>

<img src="https://user-images.githubusercontent.com/98131995/234808635-f2054c30-9136-4eb5-a04c-9f4a5fa9309e.png" width=50% height=50%><br>
![image](https://user-images.githubusercontent.com/98131995/234813931-64be6011-5b31-462b-bd37-0302cfaa437d.png)<br>
![image](https://user-images.githubusercontent.com/98131995/234808181-4b135a6c-77bd-4b5b-bde8-e42f90289162.png)<br><br>

### M3 - PRM collision free path:<br>

![PRM](https://user-images.githubusercontent.com/98131995/234813017-be1d9443-e951-4066-95d4-73d7a4ce7a37.gif) <img src="https://user-images.githubusercontent.com/98131995/234811080-91b5ee2a-4bd5-4752-a334-f14e276752ed.png" width=50% height=50%><br>
![image](https://user-images.githubusercontent.com/98131995/234811312-6b2dd8a3-109b-4952-8c66-41ad7fcb6f86.png)<br><br>

### M4 - RRT (Rapidly-Exploring Random Trees):<br>

Only Alpha:<br>
<img src="https://user-images.githubusercontent.com/98131995/234815818-9843aeee-35a6-49ed-84aa-9a3144121026.png" width=60% height=60%><br>
![image](https://user-images.githubusercontent.com/98131995/234816173-feef4c87-8c8b-48bc-a205-836cf53cd7c6.png)<br><br>

With Beta:<br>
<img src="https://user-images.githubusercontent.com/98131995/234815499-a45d37da-7e40-485b-8535-3d420bdee277.png" width=60% height=60%><br>
![image](https://user-images.githubusercontent.com/98131995/234816305-f4b58cb6-5ae1-43ff-b14b-d93ec04f7564.png)<br><br>

![RRT](https://user-images.githubusercontent.com/98131995/234817938-2a2ed893-02f2-466c-ba1f-9457eb0a0376.gif)<br><br>

### M5 - RRT with smoothing:<br>

<img src="https://user-images.githubusercontent.com/98131995/234822512-26c17ad6-daab-48e2-bf56-9598eb1ae971.png" width=60% height=60%><br>
![image](https://user-images.githubusercontent.com/98131995/234821674-354a7dcf-64d1-4809-8a9f-d6c413398aac.png)<br>
![RRT w smoothing](https://user-images.githubusercontent.com/98131995/234821001-5791d375-0421-4016-bdd0-0d84674927a2.gif)<br><br>

### M6 - More challenging implementation of RRT:<br>

This implementation has 3 objects to avoid instead of 1 while traversing from start to goal.<br>

![image](https://user-images.githubusercontent.com/98131995/234825159-57ca6a2a-e64b-4b6e-be43-5fb86965dff0.png)<br>
![RRT w 3 globes](https://user-images.githubusercontent.com/98131995/234824781-d79013b4-cc17-473e-8501-f6079b4d037d.gif)<br><br>
