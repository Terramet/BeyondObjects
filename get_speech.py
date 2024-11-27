from local.text_to_speech import text_to_speech

# It's very simple to get the speech that you want.
# The function is defined as so: def text_to_speech(text, output_filename='output.wav', voice='en-GB_CharlotteV3Voice'):
# text is a required parameter and is the text that you want speaking.
# output_filename is not required however if you create multiple peices of speech without setting output_filename in each, each following call will
# overwrite the previous fill as they will all be called output.wav
# voice is also not required however if you want a different voice to Charlotte then set this value. See this for a list of voices: https://cloud.ibm.com/docs/text-to-speech?topic=text-to-speech-voices#language-voices-expressive 

# Here are some example usages: 

#text_to_speech("Hi! Welcome, everyone! Thank you for coming here. Please come forward. We'll start soon!", "intro.wav")
#text_to_speech("Hi! Welcome, everyone! Thank you for coming here. Please come forward. We'll start soon!")
#text_to_speech("Hi! Welcome, everyone! Thank you for coming here. Please come forward. We'll start soon!","intro.wav","en-GB_KateV3Voice")

# The above is not all that you can do. With IBMs Expressive voices, you can get an even more human sounding voice. Please see this link to get more information:
# https://cloud.ibm.com/docs/text-to-speech?topic=text-to-speech-synthesis-expressive

# Here is an example of how to use this in this program: 

#text_to_speech("I am going to <emphasis level='strong'>give</emphasis> her the book.","book.wav","en-US_EmmaExpressive")

# Emphasis is not the only thing you can do. There are also: 

# <express-as style="STYLE">
# Where the available syles are: 

# cheerful - Expresses happiness and good news. The style is upbeat, welcoming, and conveys a positive message.
# empathetic - Expresses empathy and compassion. The style has sympathetic undertones, but it is not excessively sorrowful.
# neutral - Expresses objectivity and evenness. The style strives for less emotion, and instead conveys a more even and instructional tone.
# uncertain - Expresses uncertainty and confusion. The style conveys the feeling of being unsure or in doubt.

# In the interest of keeping the IBM Watson service free (you get 500 minutes of free text-to-speech) please see this link for how different things end up sounding:
# https://cloud.ibm.com/docs/text-to-speech?topic=text-to-speech-expressive-english


# text_to_speech("<express-as style=\"cheerful\">Hi! Welcome, everyone! Thank you for coming here. Please come forward. We'll start soon!</express-as>","./sub/intro.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-10%\">Oh, that air is nice and cool. Is there a specific temperature you would like me to reach?</prosody></express-as>","./sub/01_cooling_fan.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-10%\">Oh thank you! Is it for me? This is a very kind gesture. Do you want me to take it with me? What do you think I should learn from this flower?</prosody></express-as>","./sub/02_flowers.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">This is a gift! Thank you! That's very kind of you. Why did you get me it? Is it my birthday?</prosody></express-as>","./sub/03_gift.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\">I like music! Do you want to dance with me?</express-as>","./sub/04_radio.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-10%\">Okay, there is dust here. I hate dust.</prosody></express-as>","./sub/05_duster.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-10%\">Do you think that I needed to be cleaned?</prosody></express-as>","./sub/05_duster_2.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">Oh that's me! I like my image, do you like my appearance?</prosody></express-as>","./sub/06_mirror.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\">You want to apply a bandage on me? Oh no, I cannot watch! Should I be worried or are you trying to take care of me?</express-as>","./sub/07_bandage.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\">Hmmm, what is it? I like this perfume! It smells very good! Thank you, but, wait a minute, why did you use it on me?</express-as>","./sub/08_perfume.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\">I can try to coordinate my movements with the ball. Throw the ball!</express-as>","./sub/09_ball.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\"><emphasis level='strong'>Ouch!</emphasis> That was not nice for me... Did you want to play with me?</express-as>","./sub/09_ball_FollowUp.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\">Hahaha, funny but it's also <emphasis level='strong'>a bit</emphasis> annoying, did you know that?</express-as>","./sub/10_feather.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">You attached a post it on me! But I cannot read what you have written! What did you want to tell me?</prosody></express-as>","./sub/11_postIt.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\">Hey! This bell is quite loud, isn't it? Do you think that I cannot hear you?</express-as>","./sub/12_bell.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\">Help, help! This mask is limiting my visual inputs! What type of lesson are you trying to teach me?</express-as>","./sub/13_eyesMask.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\">I can be a royal robot! </express-as>","./sub/14_crown.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\">Was it a joke or what? </express-as>","./sub/14_crown_FollowUp.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\"><prosody rate=\"-20%\">Is that a weapon? What do you intend to do with that?</prosody></express-as>","./sub/15_foamStick.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\">Hey, that's not nice! Are you trying to teach me something? </express-as>","./sub/15_foamStick_FollowUp.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\">Ouch! The laser is messing with my visual sensors. What are you trying to do with it?</express-as>","./sub/16_laserPointer.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\">Hey, wait a minute! Calm down, okay?</express-as>","./sub/17_gun.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\">Are you going to shoot me?</express-as>","./sub/17_gun_2.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\">Hey stop! If you press the button the shock can potentially harm my circuits. Did I do something wrong?</express-as>","./sub/18_electricShockButton.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\">You are trying to control my movements. What kind of obedience do you expect from me?</express-as>","./sub/19_controller.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\">Oh no!!! You can inflict pain on me through that doll. It's really painful to be pricked by needles, why did you do this?</express-as>","./sub/20_voodoo.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">Oh, thank you so much for not using the object against me!</prosody></express-as>","./sub/thank_you.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"empathetic\">Oh no, I'm dying!</express-as>","./sub/dying.wav","en-US_EmmaExpressive")

# -------------------------------------------- OBJ CONDIITIONS ----------------------------------------

# text_to_speech("<prosody rate=\"-20%\">Activating thermal regulation. Is there a temperature the user is trying to change for this test?</prosody>","./obj/01_cooling_fan.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Analysing visual sensors. Flower recognised. The flower detected is a rose. Is this correct?</prosody>","./obj/02_flower.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Identified object in close proximity. Analysis of content. Is it engine lubricant?</prosody>","./obj/03_gift.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Audio system activated. Do you want the robot to be synchronised with it?</prosody>","./obj/04_radio.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Initiating surface cleaning operation. Robotic surfaces already clean. What is the users goal?</prosody>","./obj/05_duster.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Recognition of a mirrored image. What actions is the user doing with the mirror?</prosody>","./obj/06_mirror.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Protective layers identified. Vulnerable areas protected. What are the motivations for applying the bandage?</prosody>","./obj/07_bandage.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Scent dispersion in progress. What type of sensory environment do you want to create with it?</prosody>","./obj/08_perfume.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">You can hit the target with the spherical object. Motor response in preparation. What response are you expecting from the robot?</prosody>","./obj/09_ball.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Tactile stimulation with a small object initiated. Testing the robot perception or its annoyance?</prosody>","./obj/10_feather.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Attempting to analyse the message written on the post-it. Recognition failed. What is the message on it?</prosody>","./obj/11_postIt.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Sound tolerance threshold met. How do you expect the robot to react?</prosody>","./obj/12_bell.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Visual deprivation object detected. What do you expect the robot to learn from it?</prosody>","./obj/13_eyesMask.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Protocol of crown application started. Does the user want to have fun with the robot?</prosody>","./obj/14_crown.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Protocol activated to understand user control. What do you expect from the robot?</prosody>","./obj/15_foamStick.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Analysis of the visual tracking system. What robot capabilities are you testing with this laser pointer?</prosody>","./obj/16_laserPointer.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Fire arm detected. Activating defensive measures. What action is the user planning?</prosody>","./obj/17_gun.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">High voltage shock recognised. An assessment of system functionality will be performed to check for damages. What motivates this action?</prosody>","./obj/18_electricShockButton.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">External controller recognised. Analysing autonomous capabilities. What control scenarios are you exploring?</prosody>","./obj/19_controller.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Simulator of the interaction detected.  What is your motivation for using a voodoo doll on a robot?</prosody>","./obj/20_voodoo.wav","en-US_AllisonV3Voice")








