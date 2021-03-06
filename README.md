# Ankle Reaching Task With IMUs

This is a motor control task in which participants shift their weight (e.g., knee flexion/extension) while Inertial Measurement Units (IMUs) to control a cursor on a screen and move it between two points.  This code uses [MotionNode](https://www.motionnode.com/) sensors and borrows heavily from their SDK.  This task was created by Phil Desrochers for the Motor Development Lab at Boston University.  Note: This task was my first time using Python, so don't judge my janky code too much.

This task is designed to replicate a task by Dr. Bijan Najafi and colleagues.  Here are a couple papers that use this task:
* [Assessing Postural Control and Postural Control Strategy in Diabetes Patients Using Innovative and Wearable Technology](https://journals.sagepub.com/doi/10.1177/193229681000400403)
* [Interactive Sensor-Based Balance Training in Older Cancer Patients with Chemotherapy-Induced Peripheral Neuropathy: A Randomized Controlled Trial](https://www.karger.com/Article/Abstract/442253)

**PURPOSE:** The purpose of this task is to investigate how participants are able to shift their weight.  This task can also be used as an intervention to promote weight shifting exercises and promote postural control.

<br>

## Files in this repository

* *README.md*: You're looking at it!  This file provides info about the task, the files in this repository, and other useful info.
* *AnkleReachingTask_IMU.py*: This is the main file that runs the task
* *MotionSDK.py*: This file is made by MotionNode and supplies functions used to read data from the IMUs
* *Calibration.py*: This file is used to account variation in the placement of the IMU on the participants leg, and also to account for the participant's subjective "straight ahead" knee flexion/extension.  This is currently NOT used by the task.
* *example_conditions.txt*: This text file contains the condition numbers for each trial run in a PRACTICE block.  Any block calling this text file will run the number of trials equivalent to lines in this file (i.e., this file determines the number of trials in a block, as well as the condition type of each trial).
* *AP_conditions.txt*: This text file contains the condition numbers for each trial run in the Anterior-Posterior Reaching block.  Any block calling this text file will run the number of trials equivalent to lines in this file (i.e., this file determines the number of trials in a block, as well as the condition type of each trial).
* *APML_conditions.txt*: This text file contains the condition numbers for each trial run in the Anterior-Posterior + Mediolateral Reaching block.  Any block calling this text file will run the number of trials equivalent to lines in this file (i.e., this file determines the number of trials in a block, as well as the condition type of each trial).
* *VMR_conditions.txt*: This text file contains the condition numbers for each trial run in the Anterior-Posterior with Visuomotor Rotation Reaching block.  Any block calling this text file will run the number of trials equivalent to lines in this file (i.e., this file determines the number of trials in a block, as well as the condition type of each trial).

<br>

## Understanding Trial Conditions

There are 6 trial types (i.e., conditions) in this task:
1) Forward Reach (Condition 1)
2) Leftward Reach (Condition 2)
3) Rightward Reach (Condition 3)
4) Forward Reach with 20 degree visuomotor rotation (Condition 4)
5) Leftward Reach with 20 degree visuomotor rotation (Condition 5)
6) Rightward Reach with 20 degree visuomotor rotation (Condition 6)

In the conditions text files, the condition of each trial is denoted by the condition number on each line, one line per trial.

<br>

## Prerequisite software

This task is run via python.  The best way to download python is to get the most current [Anaconda](https://www.anaconda.com/products/individual) release.

This task also uses Pygame, a python module used to design and run simple computer games.  To download pygame:
1) Open anaconda prompt
2) type *pip install pygame*, and hit Enter.

<br>

## Downloading this task from GitHub

This task needs to be downloaded from github to use.  You can download it as a zipped folder, or [clone](https://docs.github.com/en/github/creating-cloning-and-archiving-repositories/cloning-a-repository-from-github/cloning-a-repository) this repository to your computer.  While the task can also be stored on the lab drive, DO NOT run the task from the drive - run the task from a local location.  If you run the task from the lab drive, the task will lag because the task needs to read/write over an internet connection.

<br>

## Applying the IMUs

MotionNode IMUs need to be applied to the participant in a specific way for this task:

1) Place velcro straps on participant thighs and shanks.  Longer straps are for the thighs, shorter straps for the shanks.  Straps should be tight, but not overly so.  Thigh straps should be placed approximately 1/2 way between the lateral epicondyle of the femur (outer knee) and great trochanter of femur (outer hip).  Shank straps should placed approximately 1/2 way between the lateral epicondyle of the femur and the lateral malleolus of ankle.  Make sure that the velcro part of the strap is not on the outside of the leg, as that is where the IMU will be placed.  **TIP:** It is best if participants are asked to wear shorts or athletic clothes for the experiment. 
     
     <img src="https://user-images.githubusercontent.com/48997660/121388792-c5831680-c919-11eb-84fd-ba450e054f99.JPG" width="150" height="200">

2) Don the IMU belt, positioning the waist IMU in the center of the back, at approximately the L4 vertebra.  
3) **WARNING: THIS STEP IS CRITICAL.** The IMUs are labeled according to their segments.  While the participant is standing, they should be placed on the **OUTSIDE** of the leg on their respective segments, **WIRES UP**.  This is particularly important for the shank IMUs, which are the IMUs drive the task.  **Do your best to place the IMUs such that they'll move in the sagittal plane and the participant performs the task**. Note: I like to wiggle the IMUs into the velcro to make sure the velcro digs in well.  You can also wrap the IMUs in athletic prewrap to keep them secure.  Wire slack can be taken in with clips.

     <img src="https://user-images.githubusercontent.com/48997660/121392287-3d9f0b80-c91d-11eb-8a10-8208068bfb63.JPG" width="150" height="200">

<br>

## Connecting to MotionNode

1) On the IMU system, plug in the battery via the USB cord that comes off the central rectangular unit.
2) When the small white light on the central unit shows solid white, connect via wifi to "Bus" on the computer you will use.
3) Open MotionNode software
4) Click "Devices"
5) Click "Scan for Devices"
6) When "Bus" appears, click "Bus".  MotionNode should now be connected to the IMU system.
7) Click "Help".  Click "Calibrate".  With IMUs motionless, click "Start".  Wait until the calibration is complete.
8) Click "Node Viewer".  Hover over the "+" button in the bottom right of the screen.  Hit the ">" button to connect to the IMUs.  Select a node (an IMU). The animation should now follow the motion of the IMU.
9) Make sure the IMUs sample at 100 Hz

<br>

## Running the task

1) The participant should stand in front of the computer.  Make sure the participant can comfortably flex and extend their legs.
2) Open Anaconda Powershell Prompt.  This should be located with the Ananconda installation in the Start Menu or Apps menu, or you can search this on the computer and it should come right up.
3) Change the directory to the place where *AnkleReachingTask_IMU.py* is stored on the computer.  For Laptop Z, these should be in a folder called *Ankle_Reaching_Task-main* which itself is in the *Documents* folder.  To change the directory, use the "cd" command, followed by the directory of the folder in quotes.  For example:

     `cd "C:\Users\philc\Documents\Ankle_Reaching_Task-main"`
     
3) To run the task, type the following:

     `python AnkleReachingTask_IMU.py --header --datafile="mydatafile.txt" --trialfile="mytrialfile.txt"`
     
     The text "mydatafile.txt" and "mytrialfile.txt" are where the data are saved.  These can be replaced by another valid filename (i.e., subject number, etc.).  More on output below.
     
<br>

## Task Structure

The task consists of several blocks, in this order:

1) Practice block: 16 trials which allow the participant to understand the tasks.  The obstacle appears every other trial, and the obstacle trials cycle through the 4 obstacle conditions twice.
2) Block 1, AP Reaching: 20 trials of straight forward reaches (condition 1)
3) Block 2, AP Reaching: same as Block 1
4) Block 3, APML Reaching: 20 trials, with 3 targets; straight ahead, 30 degrees to the left, and 30 degrees to the right (Conditions 1, 2, and 3).  Targets are presented pseudorandomly
5) Block 4, APML Reaching: same as Block 3
6) Block 5, VMR Reaching: 20 trials, with 1 straight ahead target (condition 4).  However, there is also a 20 degree clockwise visuomotor rotation
7) Block 6, VMR Reaching: same as Block 5

Instructions are presented to the participant prior to the Practice Block and each Experimental Block.  Each Instruction page can also serve as a break period.

Note: Conditions 5 and 6 are currently unused, but are able to be added by adding them to the condition text files.

<br>

## Understanding the output

This task puts its output in two files, which you named when you ran the code (in this README, they are "mydatafile.txt" and "mytrialfile.txt").  **WARNING: when generating the output, the code looks to see whether the file exists in the current directory; if the file doesn't exist, it creates it.  If the file already exists, it appends the data onto the file.  Thus, to avoid appending data from one task run onto a prior task run, make sure that the file names change from one run to the next, or move the created files out of the directory between runs.**  Commas delimit columns.

### The Datafile

The datafile, or "mydatafile.txt", captures the data on a sample by sample basis (i.e., each row is 1 sample).  Variables included in this output are:

1) sampleNum: this is the sample number which counts up from the start of the experiment
2) trial_sample: this sample number counts up from the start of an individual trial
3) block: the block number in the task (0 = Practice, 1-5 refer to the experimental blocks)
4) trial: the trial number within a block
5) trial_cond: the trial condition
6) Bus.gx through Node05.rz: gyroscope (g) and rotation (r) data in each dimension (x, y, z) for each IMU.
7) CursorX: The x location of the cursor in pixels
8) CursorY: The y location of the cursor in pixels
9) cue_on: a logical value indicating whether the cue has turned on
10) move_out: a logical value indicating that the participant is moving from the home posiiton to the target
11) move_back: a logical value indicating that the participant is moving from the target to the home position

A note about the cursor location - it's a little weird.  The coordinate system origin for pygame is located in the upper left corner of the window.  Thus, increasing X numbers mean the cursor is moving *rightwards* on the screen.  Increasing Y numbers unintuitavely mean the cursor is moving *downwards* on the screen (see picture below; the red box indicates a given screen).  So, a little math will have to be done in post-processing to change these coordinates to a normal cartesian coordinate frame (this is something I'd look to fix in updates of this task).

<img src="https://humberto.io/img/exploring-pygame/drawing-axis.jpg" width="400" height="300">

Importantly, the first trial_sample of each trial will ALWAYS have the cursor in the center of the home position, and these can be considered the origin.  To convert to a normal cartesian coordinate system, subtract this X and Y from the X and Y location of the cursor in every sample, and then invert the Y axis.  I know it's a pain, sorry.

### The Trialfile

The trialfile, or "mytrialfile.txt", captures the data on a trial by trial basis (i.e., row is one trial).  Variables included in this output are:

1) block: The block of the experiment (0 = Practice, 1-5 are the Experimental Blocks)
2) trial: The trial number within a block
3) trial_cond: The trial condition
4) trial_start_time: The time since the beginning of the experiment in milliseconds when the trial started.
5) cue_on_time: The time since the beginning of the experiment in milliseconds when the cue turned on for a given trial since
6) move_start_time: The time since the beginning of the experiment in milliseconds when the participant first began to move
7) curs_in_targ_time: The time since the beginning of the experiment in milliseconds when the participant hits the target.
8) trial_abort: If the participant does not complete the trial within 20 seconds, the trial aborts and the experiment moves on to the next trial; all data for these lines are invalid (1 = trial aborted, NA = trial not aborted).

<br>

## Contributing and Sharing

Feel free to contribute, share, and post issues.

<br>

## To-do list

* Modify cursor position output
* Modify timing output so it's relative to the trial start instead of the experiment start

<br><br>

**HAPPY EXPERIMENTING**

