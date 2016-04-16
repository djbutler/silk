---
layout: default 
title: Silk - About 
---

## Cheap, easy, convenient: For robotic telemanipulation, pick two

### Question: Can we make a robotic arm as cheap, easy, and convenient to remote control as a toy car?

Even the simplest manipulation tasks involve fairly complex sequences of commands.

One line of attack on this problem is to gather rich 3D input data from the user.

- In the area of medical surgury, people have developed very expensive joysticks with force feedback 
- In the area of video games, people have developed handheld tracking devices, like the Wiimote, Razer Hydra, or PlayStation Move, or camera-based systems like Kinect or Hololens
- In academic robotics research, people have developed a variety of mouse-based and touch-based interfaces, in addition to interfaces that take advantage of the above systems

Academic robotics research has become increasingly interested in the idea of crowdsourcing - in other words, paying human workers to control robots remotely over the internet. For these types of applications, specialized 3D input devices are typically not available.
(Raises some questions - cheap at-home webcam-based tracking? Or what about smartphone accelerometers?)

Relatively cheap telepresence robots ($1500-$2500) have been gaining traction. If telepresence robots could be augmented with manipulators, additional applications would become feasible - remote service jobs and elderly assistance, for example. For these applications, specialized input devices are a feasible solution, but would have to be cheap. And if these systems did not require special input devices so that anyone with a computer could use them, this could greatly accelerate adoption.

Prior research has looked at robot user interfaces on commodity hardware. However, significant usability and efficiency issues remain. Even simple tasks such as picking up an object can take several minutes. More complex tasks, such as pouring a glass of water or opening a door, can be even more time consuming, if they are even possible at all.

Another reason why traditional mouse-and-keyboard interfaces may be preferable to 3D input devices is that over the past 30 years they have proven very durable despite other alternatives. While the 3D aspects of manipulation may benefit from new input devices, traditional mouse-and-keyboard interfaces have proven very effective at affording and organizing discrete user actions.

This state of affairs leads to a natural question: is it possible to make more usable and efficient user interfaces for controlling robotic manipulators using commodity input devices?

New ideas and strategies are needed in order to make substantial gains.

Some fundamental strategies that we adopt are:

1. iterating rapidly on user interface concepts;
2. putting these interfaces in front of potential end-users to collect realistic evaluations;
3. using quantitative evaluation criteria (both objective and subjective) to verify that progress is being made;
4. deeply infusing the interfaces with modern techniques from computer vision and planning;
5. identifying good interactions that can drive new research goals for computer vision and planning.

We observe that prior research adopting these strategies has been both impactful and also quite rare in the domain of telemanipulation. (Leeper, Hauser, Dragan).

We are particularly interested in interfaces that take advantage of mid-level perceptual information - point clouds, planes, edges. This type of information is can be acquired more robustly in a wider variety of settings than high-level perceptual information such as object categories (e.g. distinguishing book vs. box, or apple vs. pear). In this way, we hope to improve the usability and efficiency of teleoperation without sacrificing too much of its generality.

Hierarchy of UIs based on the sophistication of the perception system:

1. no visual information
2. raw point cloud (raw Kinect, semi-dense 3D geometry)
3. fused point cloud (KinectFusion, dense 3D geometry)
4. edge / feature detection for snapping
5. grasp detection
6. object segmentation
7. object recognition

