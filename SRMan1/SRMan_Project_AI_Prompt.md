# Prompt for Building SRMan Project: Phase I

**Act as an expert in robotics, specifically in manipulator kinematics, MATLAB, and Simulink.** I need your help to complete Phase I of my university laboratory project on the Kinematic analysis of the KUKA LBR MED surgical manipulator.

## My Setup & Resources:
* **Hardware:** I am running MATLAB/Simulink on a MacBook Pro with an Apple M2 Pro chip and 32GB RAM. Please ensure any code, compilation steps, or Simulink configurations are optimized for Apple Silicon.
* **Provided Tools:** I must use a provided Symbolic Robotics Matlab Toolbox located in a folder named `Robotics_Symbolic_Matlab_Toolbox-2` as the starting point for my computational models. I also have a 3D simulation environment in a folder called `RobotX_sim3d`.
* **Course Materials:** I have several class presentations available that I can upload if you need them for reference, including `RMan - Kinematics.pdf`, `Inverse Kinematics of Robotic Manipulators.pdf`, and `Surgical Robotics.pdf`.

## Project Deliverables (Phase I):
I need to write my own code and provide complete, concise answers for the following five tasks. Please guide me through them step-by-step:

1. **DH Convention:** Analyze the KUKA LBR MED's drawing and build its kinematic model according to the Denavit-Hartenberg convention. We need to create a joints diagram with the link reference frames and the corresponding table of parameters.
2. **Direct Kinematics:** Using the provided Toolbox, create a Simulink model for the direct kinematics of the robot. We then need to validate this model by placing the robot in configurations where the end-effector's position and orientation are easily determined.
3. **Inverse Kinematics (Closed-Form):** Following a kinematic decoupling approach, present one closed-form solution for the inverse kinematics of the robot and identify the singularity conditions. Then, create the Simulink model and validate it against the direct kinematics model from Step 2. Keep in mind that the manipulator is redundant.
4. **Geometric Jacobian:** Create a Simulink model for the geometric Jacobian of the robot. We must validate the result through numerical differentiation of the direct kinematics in Step 2, and show the effect of the kinematic singularities on the rank of the Jacobian matrix.
5. **Closed Loop Inverse Kinematics (CLIK):** Using the models from Steps 2 and 4, implement a Simulink model for the Closed Loop Inverse Kinematics of the robot, specifically stabilizing the null space. We will validate this against the results of Step 3.

---
**Let's start with Step 1.** What specific information or diagrams from my course materials do you need me to upload first so we can accurately establish the Denavit-Hartenberg frames and parameter table?
