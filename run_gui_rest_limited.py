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

        # Use REST API for presentations
        self.presentation_url = "http://ari-33c/action/pal_play_presentation_from_name"
        
        # Load configuration from JSON file
        with open(json_file, 'r') as file:
            self.config = json.load(file)
        
        # Initialize Tkinter window
        self.root = tk.Tk()
        self.root.title("Robot Control Panel")

        # Timer section
        self.timer_label = tk.Label(self.root, text="Timer: 00:00", font=("Helvetica", 14), bg="#f7f7f7", pady=10)
        self.timer_label.grid(row=0, column=0, columnspan=3, sticky="ew")  # Span across the top

        self.timer_duration = 60*15  # Default timer duration in seconds
        self.timer_running = False

        # Intro button options
        self.intro_config = next((item for item in self.config if item.get("intro")), None)
        if self.intro_config:
            self.create_intro_button()

        # A dictionary to track button usage counts
        self.usage_tracker = {idx: item.get("uses") for idx, item in enumerate(self.config) if not item.get("intro")}
        
        # Create buttons based on the JSON configuration
        self.create_buttons()
        
        # Run the Tkinter main loop
        self.root.mainloop()

    def create_intro_button(self):
        intro_text = self.intro_config.get("test", "Intro")
        intro_colour = self.intro_config.get("colour", "#FFD700")
        intro_speech = self.intro_config.get("speech")
        intro_action = self.intro_config.get("action")
        intro_pres = self.intro_config.get("presentation")

        intro_button = tk.Button(
            self.root,
            text=intro_text,
            command=lambda f=intro_speech, a=intro_action, p=intro_pres: self.trigger_intro(f, a, p),
            bg=intro_colour,
            font=("Helvetica", 12, "bold"),
            width=20,
            height=2
        )
        intro_button.grid(row=1, column=0, columnspan=3, pady=10)

    def create_buttons(self):
        self.buttons = {}  # Store references to buttons for dynamic updates
        for idx, item in enumerate(self.config):
            if item.get("intro"):  # Skip intro button in the main button list
                continue

            test = item.get("test")
            speech = item.get("speech")
            action = item.get("action")
            pres = item.get("presentation")
            colour = item.get("colour", "#f0f0f0")

            # Create an intro button
            if test.lower() == "intro":
                button = tk.Button(
                    self.root,
                    text=test,
                    command=lambda f=speech, a=action, p=pres: self.trigger_intro(f, a, p),
                    width=20,  
                    height=5,   
                    bg=colour    
                )
                button.grid(row=1, column=0, padx=15, pady=15)  # Place intro button at the top row
            else:
                # Create other buttons
                button = tk.Button(
                    self.root,
                    text=test,
                    command=lambda x=idx, f=speech, a=action, p=pres: self.trigger_action(x, f, a, p),
                    width=20,  
                    height=5,   
                    bg=colour    
                )
                button.grid(row=idx // 3 + 2, column=idx % 3, padx=15, pady=15) 
            self.buttons[idx] = button  # Store the button reference

    def trigger_intro(self, speech, action, pres):
        print("Intro button clicked")  # Debugging statement
        self.trigger_action(None, speech, action, pres)
        self.start_timer()

    def start_timer(self):
        if not self.timer_running:
            self.timer_running = True
            self.current_time_left = self.timer_duration  # Ensure fresh start every time
            self.countdown()  # Start the countdown
        else:
            print("Timer is already running")  # Debugging statement

    def countdown(self):
        if self.current_time_left >= 0:
            mins, secs = divmod(self.current_time_left, 60)
            self.timer_label.config(text=f"Timer: {mins:02}:{secs:02}")
            self.current_time_left -= 1  # Decrease the time left
            self.root.after(1000, self.countdown)  # Call countdown every 1000 ms (1 second)
        else:
            self.timer_running = False
            self.timer_label.config(text="Timer: 00:00")  # Reset to 00:00 when time is up
            print("Timer finished")


    def trigger_action(self, idx, speech, action, pres):
        # Play the sound
        if speech and os.path.exists(speech):
            pygame.mixer.music.load(speech)
            pygame.mixer.music.play()

        # Send the presentation goal via REST API
        if pres is not None:
            payload = {"presentation_name": pres}
            try:
                response = requests.post(self.presentation_url, json=payload)
                if response.status_code == 201 or response.status_code == 200:
                    print(f"Presentation '{pres}' goal created.")
                else:
                    print(f"Failed to start presentation. Status code: {response.status_code}")
                    print("Response:", response.text)
            except requests.exceptions.RequestException as e:
                print("Error occurred:", e)

        # Send the action goal via REST API
        if action is not None:
            payload = {"filename": action}
            try:
                response = requests.post(self.motion_url, json=payload)
                if response.status_code == 201 or response.status_code == 200:
                    print(f"Motion '{action}' goal created.")
                else:
                    print(f"Failed to start motion. Status code: {response.status_code}")
                    print("Response:", response.text)
            except requests.exceptions.RequestException as e:
                print("Error occurred:", e)

        # Update the usage tracker and remove button if limit reached
        if idx is not None and self.usage_tracker[idx] is not None:
            self.usage_tracker[idx] -= 1
            if self.usage_tracker[idx] <= 0:
                self.buttons[idx].grid_forget()
                del self.buttons[idx]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Wizard of Oz GUI")
    parser.add_argument("-c", "--config", type=str, required=True, help="The file path to the config file that you wish to load")
    args = parser.parse_args()
    try:
        gui = DynamicGUI(args.config)
    except Exception as e:
        print(f"Error: {e}")
