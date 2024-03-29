# UWB Proximity Alert System
This project was developed for the ECE Capstone project during the semester of Fall 2023 at Michigan State University. 
Portable multi-device system developed to alert the visually impaired of oncoming scooters/bikes. The project utilizes two ESP32-UWB modules to measure the distance between the modules using UltraWideband ranging techniques.

<img src="https://github.com/ayushchinmay/ECE480-TEAM9/blob/main/rmRef/final.png" width="400">

**Team 9 Members:** *Ayush Chinmay, Vigneshwer Ramamoorthi, Kattie Romero-Otero, Shayna Wilson, Pradnya Ghorpade*

## Concept
For the purposes of this project, in the absence of fixed indoor positioning anchors, Two-Way Ranging (TWR) technology is employed to calculate distances between two devices. UWB can accurately determine the location of a device within a range of under 200 meters, with optimal performance observed within shorter ranges of 1-50 meters and under conditions of line of sight between devices or anchors. The time taken for a signal to traverse this distance is multiplied by the speed of light, forming the basis for determining their relative positions. The measured distance is then used to establish alert priorities conveyed to users.

<img src="https://github.com/ayushchinmay/ECE480-TEAM9/blob/main/rmRef/tof.png" width="400">

## Component Description
<img src="https://github.com/ayushchinmay/ECE480-TEAM9/blob/main/rmRef/flow.png" width="400">

The central communication and control unit for this project is the ESP32-UWB Pro board, manufactured by Maker-Fabs. This board integrates the ESP32-WROOM board with an on-board Decawave DW1000 UWB module. The ESP32 microcontroller manages data reading, initiation of communication events, calculation of alert priorities, and activation of the piezo-buzzer alerts.

## Final Results
The system successfully established a connection, and measured distances between the scooter and tag units through Two-Way Ranging. Verification involved commencing tests from the same point in a hallway, moving away while accurately measuring the distance traveled, and closely monitoring signal integrity. Another test, starting with the units out of range and then moving closer, demonstrated swift initial pairing once within range. The measured distance accuracy reached up to 20 cm at its best. Appropriate sonic alerts were triggered at the programmed distance intervals.

<img src="https://github.com/ayushchinmay/ECE480-TEAM9/blob/main/rmRef/proto.png" width="600">

## Achievements
The Scooter Proximity Alert System project attained first place for ECE Senior Design projects during the Fall of 2023 at Michigan State University.

<img src="https://github.com/ayushchinmay/ECE480-TEAM9/blob/main/rmRef/award.png" width="800">
[ECE Senior Design Awards -- Fall 2023](https://designday.msu.edu/ece-awards-fall-2023/)
