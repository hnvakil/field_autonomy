# field_autonomy
Comprobo final project: getting a robot to drive autonomously in the field

# proposal
What is the main idea of the project?
* We are going to retrofit an existing remote controlled vehicle into an autonomous outdoor vehicle using a system consisting of an Arduino, Raspberry Pi, smartphone running an AR app, and ROS2.

Provide some motivation for your project. Why does your team want to pursue this? What possible applications does the project have? Given our discussions in class, are there possible implications for society? (Note: We’re not expecting a lengthy analysis, but some ideas and consideration.)
* We think that this is cool and is a good systems project which aligns with our learning goals. We are also excited about working in a non-controlled environment (outdoors) and moving beyond using Neatos in the mac. Any autonomous vehicle has potential impact for harm - both in if it is poorly implemented and if it is used in a manner that has high potential for harm. We are thus choosing to pursue this while in college as an experiment where the consequences are lower.

How are you planning to engage with the Ethics and Responsible Use Statement part of the project?
* We will plan to write a guidelines of use statement for our project to clarify our intent as creaters. While this is not a perfect solution, we believe that it will help us consider the ethical implications.

What topics will you explore and what will you generate? What frameworks / algorithms are you planning to explore (do your best to answer this even if things are still fuzzy)? What is your MVP? What are your stretch goals?
* We will explore field autonomy, using computer vision as well as IMU and GPS to navigate, and systems work - connecting Arduino, phone, RasPi, and robot. Frameworks and algorithms - we plan to implement fiducial tracking and learn about optical flow. Our MVP is controlling the robot with ROS teleop and having a live camera feed. Our stretch goal is to autonomously navigate parcel B. 

Outline a rough timeline for the major milestones of your project. This will mainly be useful to refer back to as we move through the project.
* Week 1 - We have investigated each piece of the pipeline in order to identify major barriars to getting teleop working. Stretch goal is working teleop.
* Week 2 - Investigate ARkit/ARcore. Work on moving past barriers to teleop
* Week 3 - Implement ARkit/ARcore solution heavily leaning on tutorials. Have working teleop.
* Weeks 4 and 5 - Triage for what we sucessfully did not finish in our original plan.

What do you view as the biggest risks to you being successful on this project?
* System integration - there are so many pieces that we have to juggle that if one of them fails we will likely struggle a lot.

Given each of your YOGAs, in what ways is this project well-aligned with these goals, and in what ways is it misaligned? If there are ways in which it is not well-aligned, please provide a potential strategy for bringing the project and your learning goals into better alignment. There should be an individual section for each person on the team addressing the fit between the YOGA and the project topic.

* Kate: This project has a lot of potential to align with my goals. To some extent I will need to understand how the whole system works but I hope to figure out how to abstract out what I need to know about each component. The system is definitely not already prescribed in how we design it so this well aligns with my goals. My only concern is finishing the project, but I feel like that's something we are going to put a lot of attention to so that is okay.