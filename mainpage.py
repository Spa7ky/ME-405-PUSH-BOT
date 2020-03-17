## @file mainpage.py
#  @author Aaron Erickson
#  @author Garrison Walters
#  @mainpage
#  @section intro Introduction
#  In this lab was for students to create a robot that autonomously competes in a sumo bot 
#  competition. The robots were required to use a variety of sensors to interact with the environment
#  such as line sensors, time of flight sensors, accelerometers, ect. The objective of the competition is to 
#  be closest to the center of the ring at the end of the match.
#  @section sec1 Purpose
#  Familiarize students with the process of integrating sensors and hardware together
#  to make a functining robot. Programming sensors to track the environment in clever ways and
#  making reliable hardware was another primary objective.
#  @section sec2 Usage
#  The usage of this program is for sumo bot competitions. Code is optimized for detection of bots
#  from 400 mm away and that are at aproximately 3 inches tall. Individual pieces of the code can be used for 
#  position tracking with odemetry, ir sensor communication, line tracking, and oponent tracking.
#  @section sec3 Testing
#  Testing for each component was executed by commenting out other task and focusing on modular functionality before 
#  combining all modules into a functioning bot. The IR sensor was tested through the use of an IR remote. Line sensor
#  was tested by holding a white box lid up to the sensor and monitoring the output. Opponet detection was tested by 
#  waving a hand in front of the TOF sensors and observing how the motors reacted to the input. Once all the modules 
#  were created and operating together we tested our bot by placing it on the arena floor and standing in front of it
#  while scanning to test its oponent detaction.
#  @section sec4 Bugs and Limitations
#  We were extremely excited to implement active position tracking, however it took too much processing power to do 
#  all of the math calculations. The IR task had difficulty handling repeat codes and worked inconsistently.
#  We suspect we were instatiating the isr incorrectly as it continued run at max speed even in a scheduling task 
#  running every second. 