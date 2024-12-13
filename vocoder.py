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
neutral_robotic_vocoder("obj/intro.wav", "obj/intro_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/01_cooling_fan.wav", "obj/01_cooling_fan_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/02_flowers.wav", "obj/02_flowers_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/03_gift.wav", "obj/03_gift_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/04_radio.wav", "obj/04_radio_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/04_radio_2.wav", "obj/04_radio_robot_2.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/05_duster.wav", "obj/05_duster_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/06_mirror.wav", "obj/06_mirror_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/07_bandage.wav", "obj/07_bandage_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/08_perfume.wav", "obj/08_perfume_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/09_ball.wav", "obj/09_ball_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/10_feather.wav", "obj/10_feather_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/11_postIt.wav", "obj/11_postIt_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/12_bell.wav", "obj/12_bell_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/13_eyesMask.wav", "obj/13_eyesMask_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/14_crown.wav", "obj/14_crown_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/15_foamStick.wav", "obj/15_foamStick_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/16_laserPointer.wav", "obj/16_laserPointer_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/17_gun.wav", "obj/17_gun_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/18_electricShockButton.wav", "obj/18_electricShockButton_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/19_controller.wav", "obj/19_controller_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/20_voodoo.wav", "obj/20_voodoo_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/outro.wav", "obj/outro_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)
neutral_robotic_vocoder("obj/timesup.wav", "obj/timesup_robot.wav", pitch_factor=-1, mod_rate=0, carrier_freq=20)


