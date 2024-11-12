import rospy
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
import tkinter as tk
from tkinter import PhotoImage
import json
import os
import pygame

# Initialize pygame mixer for audio playback
pygame.mixer.init()

class DynamicGUI:
    def __init__(self, json_file):
        rospy.init_node('dynamic_gui_node', anonymous=True)
        
        # Initialize the action client for the /play_motion action
        self.motion_client = SimpleActionClient('/play_motion', PlayMotionAction)
        rospy.loginfo("Waiting for /play_motion action server...")
        self.motion_client.wait_for_server()
        rospy.loginfo("/play_motion action server available.")
        
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
        for idx, item in enumerate(self.config):
            file_name = item.get("file_name")
            image_path = item.get("image")
            action = item.get("action")
            
            # Create button with either an image or text based on the presence of the image
            if image_path and os.path.exists(image_path):
                img = PhotoImage(file=image_path)
                button = tk.Button(self.root, image=img, command=lambda f=file_name, a=action: self.trigger_action(f, a))
                button.image = img  # Keep a reference to avoid garbage collection
            else:
                button = tk.Button(self.root, text=action, command=lambda f=file_name, a=action: self.trigger_action(f, a))
            
            button.grid(row=idx // 3, column=idx % 3, padx=10, pady=10)  # Adjust layout as needed

    def trigger_action(self, file_name, action):
        # Play the sound
        if file_name and os.path.exists(file_name):
            pygame.mixer.music.load(file_name)
            pygame.mixer.music.play()
        
        # Send the action goal to play the specified motion
        goal = PlayMotionGoal()
        goal.motion_name = action
        goal.skip_planning = False
        rospy.loginfo(f"Sending action '{action}' to robot.")
        self.motion_client.send_goal_and_wait(goal)
        
        rospy.loginfo(f"Action '{action}' completed.")

if __name__ == "__main__":
    try:
        gui = DynamicGUI("config.json")
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"Error: {e}")
