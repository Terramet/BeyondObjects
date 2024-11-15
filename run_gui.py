import rospy
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
import tkinter as tk
from tkinter import PhotoImage
import json
import os
import pygame
import argparse
import requests

# Initialize pygame mixer for audio playback
pygame.mixer.init()

class DynamicGUI:
    def __init__(self, json_file):
        rospy.init_node('dynamic_gui_node', anonymous=True)
        
        # Initialize the action client for the /play_motion action
        self.motion_client = SimpleActionClient('/play_motion', PlayMotionAction)
        rospy.loginfo("Waiting for /play_motion action server...")
        self.motion_client.wait_for_server(rospy.Duration(5))
        rospy.loginfo("/play_motion action server available.")

        # Since the ARI SDK does not come with the /pal_play_presentation messages (for whatever reason) we have the use the REST api instead.
        self.url = "http://ari-33c/action/pal_play_presentation_from_name"
        
        
        # Load configuration from JSON file
        with open(json_file, 'r') as file:
            self.config = json.load(file)
        
        # Initialize Tkinter window
        self.root = tk.Tk()
        self.root.title("Robot Control Panel")
        
        # Create buttons based on the JSON configuration
        self.create_buttons()
        
        # Run the Tkinter main loop
        self.root.mainloop()

    def create_buttons(self):
        print(f"Creating buttons")
        for idx, item in enumerate(self.config):
            test = item.get("test")
            speech = item.get("speech")
            action = item.get("action")
            pres = item.get("presentation")

            
            # Create button with either an image or text based on the presence of the image
            button = tk.Button(self.root, text=test, command=lambda f=speech, a=action, p=pres: self.trigger_action(f, a, p))
            
            button.grid(row=idx // 3, column=idx % 3, padx=10, pady=10)  # Adjust layout as needed

    def trigger_action(self, speech, action, pres):
        # Play the sound
        if speech and os.path.exists(speech):
            pygame.mixer.music.load(speech)
            pygame.mixer.music.play()
        
        if pres is not None:
            payload = {"presentation_name": pres}
            try:
                response = requests.post(self.url,json=payload)
                if response.status_code == 201 or response.status_code == 200:
                    print(f"Presentation '{pres}' goal created.")
                else:
                    print(f"Failed to start presentation. Status code: {response.status_code}")
                    print("Response:", response.text)
            except requests.exceptions.RequestException as e:
                print("Error occurred:", e)
            
            rospy.loginfo(f"Presentation '{pres}' goal created, code finished.")

        # Send the action goal to play the specified motion
        if action is not None:
            goal = PlayMotionGoal()
            goal.motion_name = action
            goal.skip_planning = False
            rospy.loginfo(f"Sending action '{action}' to robot.")
            self.motion_client.send_goal(goal)
            
            rospy.loginfo(f"Action '{action}' completed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Wizard of Oz GUI")
    parser.add_argument("-c", "--config", type=str, required=True, help="The file path the the config file that you wish to load")
    args = parser.parse_args()
    try:
        gui = DynamicGUI(args.config)
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"Error: {e}")
