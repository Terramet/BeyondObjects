import tkinter as tk
import json
import os
import pygame
import argparse
import requests

# Initialize pygame mixer for audio playback
pygame.mixer.init()

class DynamicGUI:
    def __init__(self, json_file):        
        self.motion_url = "http://ari-33c/action/motion_manager"

        # Since the ARI SDK does not come with the /pal_play_presentation messages (for whatever reason) we have the use the REST api instead.
        self.presentation_url = "http://ari-33c/action/pal_play_presentation_from_name"
        
        
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
            colour = item.get("colour", "#f0f0f0")

            # Create button with either an image or text based on the presence of the image
            button = tk.Button(
                self.root,
                text=test,
                command=lambda f=speech, a=action, p=pres: self.trigger_action(f, a, p),
                width=20,  # Set button width
                height=5,   # Set button height
                bg=colour    # Set background color
            )
            
            button.grid(row=idx // 3, column=idx % 3, padx=15, pady=15)  # Adjust layout spacing


    def trigger_action(self, speech, action, pres):
        # Play the sound
        if speech and os.path.exists(speech):
            pygame.mixer.music.load(speech)
            pygame.mixer.music.play()
        
        if pres is not None:
            payload = {"presentation_name": pres}
            try:
                response = requests.post(self.presentation_url,json=payload)
                if response.status_code == 201 or response.status_code == 200:
                    print(f"Presentation '{pres}' goal created.")
                else:
                    print(f"Failed to start presentation. Status code: {response.status_code}")
                    print("Response:", response.text)
            except requests.exceptions.RequestException as e:
                print("Error occurred:", e)

        # Send the action goal to play the specified motion
        if action is not None:
            payload = {"filename": action}
            try:
                response = requests.post(self.motion_url,json=payload)
                if response.status_code == 201 or response.status_code == 200:
                    print(f"Motion '{action}' goal created.")
                else:
                    print(f"Failed to start motion. Status code: {response.status_code}")
                    print("Response:", response.text)
            except requests.exceptions.RequestException as e:
                print("Error occurred:", e)
            
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
