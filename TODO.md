---
layout: default 
title: Silk TODOs
---

### Priorities

roslibjs's interactive markers with Silk's IK 
Axis snapping (Sketchup-style)
camera-plane dragging (with planar guide)

Leeper condition 3 (click to init pose)
Snap-to-world with a fake mesh scene
User controls wrist position, gripper orients to "reach" for closest scene point

think about "challenge problem" like pouring a cup of tea
how to display the option to perform complex actions like grasping, and render plan to show when likely to fail?

make interactive marker + IK work with UR5 and Fetch
control only 1 or 2 joints at a time (for instance, hover link and its parent) 

put a few IK regularization options in a menu (function type, function params, scaling/weighting) 
Multi-touch IK (naive is probably not useful, but fun exercise!) 

### Evaluation

the docking task
a game

### Misc
- Multiple views for comparing interactions side-by-side

### Viewpoint Experiments 
- Multiple viewpoints simultaneously (wide, zoomed-in, and )
- Viewpoint changes to be perpendicular to controlled degree of freedom
- Double tap on scene element to set camera focus (like iOS Safari) 
- Tilt device to swivel camera 
- Buttons for different camera perspectives 
- Render scene from robot's perspective 

### Probably won't impl 
- Render textured KinectFusion mesh 
- Render pointcloud (roslibjs shader) 
- Concealing missing parts of pointcloud ("fog of war") 
- "Arm flight control" using device tilt 
- Physics sim of PR2 (https://github.com/jicksta/threejs-collada-physics/blob/physijs/demo.html) 
- Use physics sim to control PR2 
- Storyboarding of motions (panels, motion arrows) 
