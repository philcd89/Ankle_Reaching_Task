# -*- coding: utf-8 -*-
"""
Created on Tue Jan 26 14:01:07 2021
@author: philc
This code is a first attempt at piping MotionNode data from the path into a txt file, while also driving the task
"""

#%% Setup

import argparse
import sys
import pygame
import math as m
import numpy as np
import Calibration

from win32api import GetSystemMetrics as SysMet

from xml.etree.ElementTree import XML

from scipy.spatial.transform import Rotation as R

import MotionSDK

#%% Get trial Conditions

# Get trial conditions
condfile = open("example_conditions.txt", "r")
example_conditions = np.asarray(list(map(int, condfile.readlines())))
condfile = open("AP_conditions.txt", "r")
AP_conditions = np.asarray(list(map(int, condfile.readlines())))
condfile = open("APML_conditions.txt", "r")
APML_conditions = np.asarray(list(map(int, condfile.readlines())))
condfile = open("VMR_conditions.txt", "r")
VMR_conditions = np.asarray(list(map(int, condfile.readlines())))



#%% Initialize Screen and Set target parameters

#initialize colors
black = (0,0,0)
white = (255, 255, 255)
red = (255, 0, 0)
green = (0, 255, 255)

#initialize game screen
width = SysMet(0)
height = round(SysMet(1)*0.92) # 95-ish% of screen height, to allow for window bar to be seen
screen = pygame.display.set_mode((width, height))

# Set constants
target_angle = 30 # Angle from straight reach
centerX = round(width/2)
centerY = round(height/2)
HomeX = round(width/2)
HomeY = round(height*0.75) # 3/4 down screen from bottom
HomeRad = 20
Targ1X = HomeX
Targ1Y = HomeY - 400 #Central Target
Targ2X = HomeX - round(400*m.cos(np.deg2rad(90-target_angle)))
Targ2Y = HomeY - round(400*m.sin(np.deg2rad(90-target_angle))) # Left Target
Targ3X = HomeX + round(400*m.cos(np.deg2rad(90-target_angle)))
Targ3Y = HomeY - round(400*m.sin(np.deg2rad(90-target_angle))) # Right Target
TargRad = 20
CursRad = 12

pygame.init()
pygame.display.set_caption("Ankle Reaching Task")
# icon = pygame.image.load("imagename.png")
# pygame.display.set_icon(icon)
clock = pygame.time.Clock()

font = pygame.font.SysFont("Arial", 28)

#%% Define object parameters

def Blank_Screen(color):
    screen.fill(color)  
    
def Instructions(x = centerX, y = centerY):
    
    screen.fill(black)
    
    text = font.render("Instructions", True, white)
    screen.blit(text, (x - text.get_rect().width/2, y-200))
 
    text = font.render("Welcome to the experiment!", True, white)
    screen.blit(text, (x - text.get_rect().width/2, y-140))
    
    text = font.render("In this task, you will be asked to move a WHITE CURSOR from a HOME POSITION to a TARGET", True, white)
    screen.blit(text, (x - text.get_rect().width/2, y-80))
    
    text = font.render("using the movement of your LEGS and TRUNK.  Your goal is to move the cursor STRAIGHT, FAST, and ACCURATELY", True, white)
    screen.blit(text, (x - text.get_rect().width/2, y-50))
    
    text = font.render("from the home position to the target.", True, white)
    screen.blit(text, (x - text.get_rect().width/2, y-20))
    
    text = font.render("Please try to keep your feet flat on the floor and approximately shoulder width apart throughout the task.", True, white)
    screen.blit(text, (x - text.get_rect().width/2, y+40))
    
    text = font.render('Between trials, text will appear that says "Get Ready..." and will count down "...3...2...1".', True, white)
    screen.blit(text, (x - text.get_rect().width/2, y+100))
    
    text = font.render("During this time, feel free to move your body to a comfortable neutral position in preparation for the next trial.", True, white)
    screen.blit(text, (x - text.get_rect().width/2, y+130))
    
    text = font.render("Press SPACEBAR to continue....", True, white)
    screen.blit(text, (x - text.get_rect().width/2, y+190))
    
    pygame.display.update()

def HomePos(HomeX, HomeY, HomeRad, color):
    # Home Position Parameters
    pygame.draw.circle(screen, color, (HomeX, HomeY), HomeRad)
    
def TargPos(TargX, TargY, TargRad, color):
    # Target Parameters
    pygame.draw.circle(screen, color, (TargX, TargY), TargRad)

def CursPos(x, y, CursRad, color):
    # Cursor Parameters
    pygame.draw.circle(screen, color, (x, y), CursRad)    
    

    
  
#%% IMU Control

def parse_name_map(xml_node_list):
    name_map = {}

    tree = XML(xml_node_list)

    # <node key="N" id="Name"> ... </node>
    list = tree.findall(".//node")
    for itr in list:
        name_map[int(itr.get("key"))] = itr.get("id")

    return name_map

def stream_data_to_csv(args, out, out2, theta, conditions, block):
    
    #%% INSTRUCTIONS
    run = True
    finished_Instructions = False

    #Instructions
    while run:
    
        # persistent parameters should stay in while loop
            
        # --------- GET EVENTS ---------- 
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit("User quit game")
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    finished_Instructions = True
        
        
        # --------- INSTRUCTIONS -----------
        
        # first, show some instructions
        Instructions()
        
        if finished_Instructions:
            Blank_Screen(black)
            pygame.display.update() 
            pygame.time.wait(2000)
            run = False
    
    
    client = MotionSDK.Client(args.host, args.port)

    #
    # Request the channels that we want from every connected device. The full
    # list is available here:
    #
    #   https://www.motionshadow.com/download/media/configurable.xml
    #
    # Select the local quaternion (Lq) and positional constraint (c)
    # channels here. 8 numbers per device per frame. Ask for inactive nodes
    # which are not necessarily attached to a sensor but are animated as part
    # of the Shadow skeleton.
    #
    
    # ADDING linear acceleration (la) and euler angle (r) set!!!!  This should add 6 more numbers per device per frame.
    xml_string = \
        "<?xml version=\"1.0\"?>" \
        "<configurable inactive=\"1\">" \
        "<g/>" \
        "<r/>" \
        "</configurable>"

    if not client.writeData(xml_string):
        raise RuntimeError(
            "failed to send channel list request to Configurable service")

    num_frames = 0
    sample = 1
    xml_node_list = None   
    
    # ------------ SET PYGAME CONSTANTS AND INITIAL STATES --------------
    
    # Pygame stuff
    main_loop_start_time = pygame.time.get_ticks()
    
    # Cursor Control
    xGain = 0.2
    yGain = 0.2
    
    # Colors
    homeColor = red
    targColor = black
    cursColor = white
    
    # Trials and Samples
    sample = 1
    trial_sample = 1
    trial = 1
    trialOut = []
    
    #Timing
    curs_in_targ_time = 0
    move_start_time = 0
    wait_in_home = 5000
    hold_in_targ = 300
    wait_in_targ = 2000
    curs_in_home_time = 0
    cue_on_time = 1000000
    
    # Initial States
    curs_in_home = False
    curs_in_targ = False
    targ_hit = False
    curs_in_home_prev = False
    curs_in_targ_prev = False
    targ_hit_prev = False
    move_back_prev = False
    move_back = False
    move_out = False
    cue_on = False
    cue_on_prev = False
    trial_cond = 0
    fill_color = black
    
    while True:
        
        # Block, waiting for the next sample.
        data = client.readData()
        if data is None:
            raise RuntimeError("data stream interrupted or timed out")
            break

        if data.startswith(b"<?xml"):
            xml_node_list = data
            continue

        container = MotionSDK.Format.Configurable(data)

        #
        # Consume the XML node name list. If the print header option is active
        # add that now.
        #
        if xml_node_list:
            if args.header:
                ChannelName = [
                    "gx", "gy", "gz",
                    "rx", "ry", "rz"
                ]

                name_map = parse_name_map(xml_node_list)

                flat_list = []
                for key in container:
                    if key not in name_map:
                        raise RuntimeError(
                            "device missing from name map, unable to print "
                            "header")

                    item = container[key]
                    if len(ChannelName) != item.size():
                        raise RuntimeError(
                            "expected {} channels but found {}, unable to "
                            "print header".format(
                                len(ChannelName), item.size()))

                    name = name_map[key]
                    for channel in ChannelName:
                        flat_list.append("{}.{}".format(name, channel))

                if not len(flat_list):
                    raise RuntimeError(
                        "unknown data format, unabled to print header")

                headerOut = ",".join(["{}".format(v) for v in flat_list])
                headerOut = "sampleNum," + "trial_sample," + "block," + "trial," + "trial_cond," + headerOut + "," + "CursorX," + "CursorY," + "cue_on," + "move_out," + "move_back" + "\n"
                if block == 0:
                    out.write(headerOut)
                
                # out.write(
                #     ",".join(["{}".format(v) for v in flat_list]))
                
                # ---------- creating trialOut header here as well ----------
                trialOut_header = ",".join(["block", "trial", "trial_cond", "trial_start_time", "cue_on_time", "move_start_time", "curs_in_targ_time", "trial_abort"])
                trialOut_header = trialOut_header + "\n"
                if block == 0:
                    out2.write(trialOut_header)
                
            xml_node_list = None

        #
        # Make an array of all of the values, in order, that are part of one
        # sample. This is a single row in the output.
        #
        flat_list = []
        for key in container:
            item = container[key]
            for i in range(item.size()):
                flat_list.append(item.value(i))

        if not len(flat_list):
            raise RuntimeError("unknown data format in stream")

        # ###################### PYGAME CODE ######################
        
        if trial_sample == 1:
            trial_start_time = pygame.time.get_ticks()
            
        sample += 1
        trial_sample += 1
        
        # Get some constants for this loop
        pygame.mouse.set_visible(False)
        screen.fill(fill_color)
        current_time = pygame.time.get_ticks()
        
        
        
        
        # ---------- SET PRIORS ----------
        
        # Set prior states
        curs_in_home_prev = curs_in_home
        curs_in_targ_prev = curs_in_targ
        targ_hit_prev = targ_hit
        move_back_prev = move_back
        cue_on_prev = cue_on
            
        
        
        
        # ---------- SET TRIAL CONDITION -----------
        # if trial == len(conditions)+1:
        #     break
        trial_cond = conditions[trial-1]
        
        if trial_cond == 1 or trial_cond == 4:
            TargX_cur = Targ1X
            TargY_cur = Targ1Y
        elif trial_cond == 2 or trial_cond == 5:
            TargX_cur = Targ2X
            TargY_cur = Targ2Y
        elif trial_cond == 3 or trial_cond == 6:
            TargX_cur = Targ3X
            TargY_cur = Targ3Y
        
        if trial_cond == 1 or trial_cond == 2 or trial_cond == 3:
            theta2 = 0
        elif trial_cond == 4 or trial_cond == 5 or trial_cond == 6:
            theta2 = 20
            
            
        
            
        
        # ------------ CURSOR CONTROL ----------------

        # Use raw gyroscope readings (degrees/sec)
        # FOR IMU PLACED ON SIDE OF SHANK: 
        # X controlled by z gyroscope (flat_list[20])
        # Y controlled by y gyroscope (flat_list[19])
        
        if trial_sample < 100: # This allows participant to get home position comfortable for first 3000 ms
            CursX_raw = HomeX
            CursY_raw = HomeY
            targColor = black
            homeColor = black
            cursColor = black
            text = font.render("Get set...3", True, white)
            screen.blit(text, (centerX - text.get_rect().width/2, centerY))
        elif trial_sample < 200:
            CursX_raw = HomeX
            CursY_raw = HomeY
            targColor = black
            homeColor = black
            cursColor = black
            text = font.render("Get set...2", True, white)
            screen.blit(text, (centerX - text.get_rect().width/2, centerY))
        elif trial_sample < 300:
            CursX_raw = HomeX
            CursY_raw = HomeY
            targColor = black
            homeColor = black
            cursColor = black
            text = font.render("Get set...1", True, white)
            screen.blit(text, (centerX - text.get_rect().width/2, centerY))
        elif trial_sample == 300:
            CursX_raw = HomeX
            CursY_raw = HomeY
            targColor = black
            homeColor = red
            cursColor = white
        else:
            CursX_raw += flat_list[20]*xGain
            CursY_raw += flat_list[19]*yGain
            
        # Perform rotation based on calibrated direction
        CursX_calib = CursX_raw - HomeX   # Translate to origin
        CursY_calib = CursY_raw - HomeY
        
        CursX_calib = (CursX_calib * m.cos(theta)) - (CursY_calib * m.sin(theta))     # rotate with 2d rotation matrix
        CursY_calib = (CursX_calib * m.sin(theta)) + (CursY_calib * m.cos(theta))
        
        CursX_calib = int(round(CursX_calib + HomeX))   # Translate back to home position
        CursY_calib = int(round(CursY_calib + HomeY))
        
        # Peform rotation Based on visuomotor rotation based on condition (4, 5, 6)
        CursX = CursX_calib - HomeX   # Translate to origin
        CursY = CursY_calib - HomeY
        
        CursX = (CursX * m.cos(np.deg2rad(theta2))) - (CursY * m.sin(np.deg2rad(theta2)))     # rotate with 2d rotation matrix
        CursY = (CursX * m.sin(np.deg2rad(theta2))) + (CursY * m.cos(np.deg2rad(theta2)))
        
        CursX = int(round(CursX + HomeX))   # Translate back to home position
        CursY = int(round(CursY + HomeY))
        
        # Determine distances
        dist_to_home = m.sqrt((abs(CursX-HomeX)**2) + (abs(CursY-HomeY)**2))
        dist_to_targ = m.sqrt((abs(CursX-TargX_cur)**2) + (abs(CursY-TargY_cur)**2))
        
        
        
        
        
        # --------- IS THE CURSOR IN THE TARGET(S)? ---------
        if dist_to_home < (CursRad + HomeRad):
            curs_in_home = True
        else:
            curs_in_home = False
            
        if dist_to_targ < (CursRad + TargRad):
            curs_in_targ = True
        else:
            curs_in_targ = False
            
            
            
            
            
        # --------- LOOK FOR STATE CHANGES, GET TIME, AND CREATE EVENT ---------
        if curs_in_home and not curs_in_home_prev:
            curs_in_home_time = pygame.time.get_ticks()
            
        if not curs_in_home and curs_in_home_prev:
            move_start_time = pygame.time.get_ticks()
            
        if curs_in_targ and not curs_in_targ_prev:
            curs_in_targ_time = pygame.time.get_ticks()
            
            
        
        # ----------- MANAGE EVENTS -------------------
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit("User quit game")
        
        
        
        
        # ----------- MAIN TRIAL LOOP ------------
        
        # Participant waits in home position, after period of time, target turns green to cue movement
        if curs_in_home and (current_time - curs_in_home_time > wait_in_home):
            cue_on = True
            targColor = red
            homeColor = black
            
        
        # Determine if the participant has begun to reach
        if not move_back and not curs_in_home and not curs_in_targ and cue_on:
            move_out = True
            
        # Participant must briefly hold in target, and signal move_out is over
        if curs_in_targ and (current_time - curs_in_targ_time > hold_in_targ):
            #play a sound?
            targ_hit = True
            move_out = False
            move_back = True
            
        # After target is hit, wait a period of time before turning target back to red
        if targ_hit and (current_time - curs_in_targ_time > wait_in_targ):
            targColor = black
            homeColor = red
            cue_on = False
                    
        # Once participant leaves target, turn targ_hit to False
        if not move_out and move_back and not curs_in_home and not curs_in_targ:
            targ_hit = False
            
        # If participant is once again in home position, change move_back to False
        if curs_in_home and move_back and (current_time - curs_in_home_time > 500):
            move_back = False
        
        if not move_back and move_back_prev: #if move_back switches from true to false, signalling the end of the trial, advance the trial number
            trial_sample = 1
            move_back = False
            trialOut = [block, trial, trial_cond, trial_start_time, cue_on_time, move_start_time, curs_in_targ_time]
            trialOut = ",".join(["{}".format(str(round(v, 8))) for v in trialOut])
            trialOut = trialOut + "\n"
            out2.write(trialOut)
            trial += 1
            
                
        
        
        

        # --------- UPDATE DISPLAY ----------    
        
        HomePos(HomeX, HomeY, HomeRad, homeColor)
        TargPos(TargX_cur, TargY_cur, TargRad, targColor)
        CursPos(CursX, CursY, CursRad, cursColor)
        
        pygame.display.update()
        
        # ------------ SCOPE ------------
        print("Trial: " + str(trial) + "; current_time: " + str(current_time) + "; trial_start_time: " + str(trial_start_time) + "; current_time - trial_start_time: " + str(current_time-trial_start_time))
        
        
        
        
        # ------------- ABORT TRIAL -------------
        if cue_on and not cue_on_prev:
            cue_on_time = pygame.time.get_ticks()
        
        if current_time - cue_on_time > 15000 and cue_on:
            trial_sample = 1
            targ_hit = False
            cue_on = False
            curs_in_home = False
            move_out = False
            move_back = False
            trial_abort = True
            trialOut = [block, trial, trial_cond, trial_start_time, cue_on_time, move_start_time, curs_in_targ_time, trial_abort]
            trialOut = ",".join(["{}".format(str(round(v, 8))) for v in trialOut])
            trialOut = trialOut + "\n"
            out2.write(trialOut)
            trial += 1
            trial_abort = False
            cue_on_time = 1000000
        elif current_time - trial_start_time > 20000 and not cue_on: # 15 seconds
            trial_sample = 1
            targ_hit = False
            cue_on = False
            curs_in_home = False
            move_out = False
            move_back = False
            trial_abort = True
            trialOut = [block, trial, trial_cond, trial_start_time, cue_on_time, move_start_time, curs_in_targ_time, trial_abort]
            trialOut = ",".join(["{}".format(str(round(v, 8))) for v in trialOut])
            trialOut = trialOut + "\n"
            out2.write(trialOut)
            trial += 1
            trial_abort = False
        
        
        
                
        # ------------ SAVE DATA --------------
        
        dataOut = ",".join(["{}".format(round(v, 8)) for v in flat_list])
        dataOut = str(sample) + "," + str(trial_sample) + "," + str(block) + "," + str(trial) + "," + str(trial_cond) + "," + dataOut + "," + str(CursX) + "," + str(CursY) + "," + str(cue_on) + "," + str(move_out) + "," + str(move_back) + "\n"
        
        out.write(dataOut)

        if args.frames > 0:
            num_frames += 1
            if num_frames >= args.frames:
                break
            
        if trial == len(conditions)+1:
            break

#%% Define input args
            
def main(argv, theta, conditions, block):
    parser = argparse.ArgumentParser(
        description="")

    parser.add_argument(
        "--datafile",
        help="output file",
        default="")
    parser.add_argument(
        "--trialfile",
        help="output file",
        default="")
    parser.add_argument(
        "--frames",
        help="read N frames",
        type=int, default=0)
    parser.add_argument(
        "--header",
        help="show channel names in the first row",
        action="store_true")
    parser.add_argument(
        "--host",
        help="IP address of the Motion Service",
        default="127.0.0.1")
    parser.add_argument(
        "--port",
        help="port number address of the Motion Service",
        type=int, default=32076)

    args = parser.parse_args()

    if args.datafile or args.trialfile:
        with open(args.datafile, 'a') as f1, open(args.trialfile, "a") as f2:
            stream_data_to_csv(args, f1, f2, theta, conditions, block)
    else:
        stream_data_to_csv(args, sys.stdout, theta, conditions, block)
        
#%% RUN THIS THANNGGGG
if __name__ == "__main__":
    #screen, theta = Calibration.Compute_Calib_Theta()
    main(sys.argv, theta = 0, conditions = example_conditions, block = 0)
    main(sys.argv, theta = 0, conditions = AP_conditions, block = 1)
    main(sys.argv, theta = 0, conditions = AP_conditions, block = 2)
    main(sys.argv, theta = 0, conditions = APML_conditions, block = 3)
    main(sys.argv, theta = 0, conditions = APML_conditions, block = 4)
    main(sys.argv, theta = 0, conditions = VMR_conditions, block = 5)
    main(sys.argv, theta = 0, conditions = VMR_conditions, block = 6)
    pygame.quit()
