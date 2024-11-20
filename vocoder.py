import numpy as np
import librosa
import soundfile as sf

def neutral_robotic_vocoder(input_file, output_file, pitch_factor=0, mod_rate=20, carrier_freq=100):
    """
    Applies a vocoder effect to make the voice neutral and robotic.

    Parameters:
        input_file (str): Path to the input sound file.
        output_file (str): Path to save the vocoded sound.
        pitch_factor (float): Shift in pitch (0 for neutral pitch).
        mod_rate (float): Frequency of amplitude modulation (in Hz).
        carrier_freq (float): Frequency of the carrier wave for robotic effect.
    """
    # Load the audio file
    y, sr = librosa.load(input_file, sr=None)

    # Pitch shift to neutralize gender cues (e.g., by reducing pitch variations)
    if pitch_factor != 0:
        y = librosa.effects.pitch_shift(y=y, sr=sr, n_steps=pitch_factor)

    # Generate a carrier signal (robotic constant tone)
    t = np.linspace(0, len(y) / sr, len(y))
    carrier_signal = np.sin(2 * np.pi * carrier_freq * t)

    # Modulate the audio with the carrier signal (multiplicative modulation)
    y_carrier_modulated = y * carrier_signal

    # Add amplitude modulation to mimic robotic voice
    mod_signal = 0.5 * (1 + np.sin(2 * np.pi * mod_rate * t))
    y_modulated = y_carrier_modulated * mod_signal

    # Normalize the output
    y_modulated = y_modulated / np.max(np.abs(y_modulated))

    # Save the processed audio
    sf.write(output_file, y_modulated, sr)
    print(f"Enhanced robotic vocoded audio saved to {output_file}")

# Example Usage
# Replace 'input.wav' with your input file and 'output.wav' for the output file
input_file = "robot/01_cooling_fan.wav"  # Path to your input file
output_file = "robot/01_cooling_fan_robot.wav"  # Path to save the robotic sound

# Apply the vocoder effect
neutral_robotic_vocoder("obj/01_cooling_fan.wav", "obj/01_cooling_fan_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/02_flowers.wav", "obj/02_flowers_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)

