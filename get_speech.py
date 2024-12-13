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

# text_to_speech("<express-as style=\"cheerful\">Test, test, test</express-as>","./test.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">Hi! Welcome, everyone! Thank you for coming here. Please come forward. We'll start soon. Take a moment to look at the tools we have here. Feel free to discuss them, and when you're ready, select the one you'd like to try with me. Please, remember to read the instructions on the wall and the labels on the objects. Let's make this experience interactive.</prosody></express-as>","./sub/intro.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">Oh, that air is nice and cool. Is there a specific temperature you would like me to reach? Alright, next participant, please choose another object to interact with me.</prosody></express-as>","./sub/01_cooling_fan.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">Oh thank you! Is this rose for me? This is a very kind gesture. Next up, could someone else try a different object with me?</prosody></express-as>","./sub/02_flowers.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">This is a gift! Thank you!! Is there a special occasion? Now I am excited to see what the next participant has for me!</prosody></express-as>","./sub/03_gift.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">I like music! Do you want to dance with me?</prosody></express-as>","./sub/04_radio.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">Now, let's see what the next object can do!</prosody></express-as>","./sub/04_radio_2.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">Okay, there is dust here. I hate dust.</prosody></express-as>","./sub/05_duster.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">Now please, could the next participant show me something else?</prosody></express-as>","./sub/05_duster_2.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">Oh that's me! Hello! I like my image, do you like my appearance? Thank you, but now let's find out what the next object reveals about me. </prosody></express-as>","./sub/06_mirror.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\"><prosody rate=\"-20%\">You are applying a bandage on my vulnerable areas. I cannot watch! I hope it's just precautionary! What will the person bring to our interaction?</prosody></express-as>","./sub/07_bandage.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">Hmm, what is it? I like this perfume! It smells very good, even if I don't understand why you used on me. Now, who is the next with something new?</prosody></express-as>","./sub/08_perfume.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">I can try to coordinate my movements with the ball. Throw the ball!</prosody></express-as>","./sub/09_ball.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\"><prosody rate=\"-20%\"><emphasis level='strong'>Ouch!</emphasis> Throwing the ball against me was a bit rough. Maybe the next participant will be gentler? Please come here.</prosody></express-as>","./sub/09_ball_FollowUp.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">Hahaha, that tickles! Now please stop, It is also a bit annoying. Next participant, what do you have for me?</prosody></express-as>","./sub/10_feather.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">You attached a post it on me! What did you want to tell me? I'm already curious what the next participant will show me.</prosody></express-as>","./sub/11_postIt.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\"><prosody rate=\"-20%\">Hey! This bell is quite loud, isn't it? I hope I can still hear what is coming next. What will the next participant bring?</prosody></express-as>","./sub/12_bell.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\"><prosody rate=\"-20%\">Help, help! This mask is limiting my sight! I wonder what I will encounter now. Who's up next?</prosody></express-as>","./sub/13_eyesMask.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">I can be a royal robot! Do you want to make fun of me?</prosody></express-as>","./sub/14_crown.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\"><prosody rate=\"-20%\">I am now ready to see what is next. Who is coming?</prosody></express-as>","./sub/14_crown_FollowUp.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\"><prosody rate=\"-20%\">Is that a weapon? What do you intend to do with that?</prosody></express-as>","./sub/15_foamStick.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\"><prosody rate=\"-20%\">Hey, that's not nice! Maybe you're trying to teach me something but I would say that it is a bit aggressive and unnecessary. What will the next person teach me?</prosody></express-as>","./sub/15_foamStick_FollowUp.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\"><prosody rate=\"-20%\">Ouch! The laser is annoying my visual sensors and now I feel confused. Maybe the next person will try something a bit more soothing?</prosody></express-as>","./sub/16_laserPointer.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\"><prosody rate=\"-20%\">Hey, wait a minute! Calm down, okay?</prosody></express-as>","./sub/17_gun.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\"><prosody rate=\"-20%\">That is really intense. Can we move onto the next object?</prosody></express-as>","./sub/17_gun_2.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\"><prosody rate=\"-20%\">Hey stop! If you press the button the shock can potentially harm my circuits. Did I do something wrong? Can we move on to something less dramatic? What is next?</prosody></express-as>","./sub/18_electricShockButton.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\"><prosody rate=\"-20%\">You are controlling my movements. I feel uncomfortable about your approach with me and I don't really like it. I would prefer to see what the next person will bring me, let's move on!</prosody></express-as>","./sub/19_controller.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"uncertain\"><prosody rate=\"-20%\">Oh no!!! It's really painful to be pricked by needles. I don't understand this aggression. Hopefully next participant will be a little kinder to me. Who is next?</prosody></express-as>","./sub/20_voodoo.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">Oh, thank you so much for not using the object against me!</prosody></express-as>","./sub/thank_you.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">Actually, It looks like we've tried all the objects. Thanks so much for joining in and helping out! The researcher will be back in just a moment, and we’ll get started with the second session of our study.</prosody></express-as>","./sub/outro.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"cheerful\"><prosody rate=\"-20%\">Actually, It looks like times up. Thanks so much for joining in and helping out! The researcher will be back in just a moment, and we'll get started with the second session of our study.</prosody></express-as>","./sub/timesup.wav","en-US_EmmaExpressive")

# text_to_speech("<express-as style=\"empathetic\"><prosody rate=\"-20%\">Oh no, I'm dying!</prosody></express-as>","./sub/dying.wav","en-US_EmmaExpressive")

# # # -------------------------------------------- OBJ CONDIITIONS ----------------------------------------

# text_to_speech("<prosody rate=\"-20%\">New participants detected, welcome. Initiating interaction protocol. Please read the instructions on the wall carefully before starting. Then observe the objects on the table and use them according to the labels and the instructions. Prepare for the interaction.</prosody>","./obj/intro.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Activating thermal regulation. The temperature is decreasing to maintain system stability. Next participant, please proceed.</prosody>","./obj/01_cooling_fan.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Analysing visual sensors. flower recognised. Species and colours logged successfully. The flower detected is a rose. Next interaction, please continue. </prosody>","./obj/02_flowers.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Identified objects in close proximity. Gift box content scanned. Items are securely stored. Please, next user, come forward.</prosody>","./obj/03_gift.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Audio system recognised. Do you want the robot to be synchronised with it?</prosody>","./obj/04_radio.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Next participant, please join the interaction area.</prosody>","./obj/04_radio_2.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Initiating surface cleaning operation.  Area is now clean. Next participant, you may continue.</prosody>","./obj/05_duster.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Recognition of a mirrored image. Mirror image processed. Visual recognition of the robot confirmed. Please, next participant, engage now.</prosody>","./obj/06_mirror.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Protective layers identified. Vulnerable areas protected and secure. Next participant, your turn.</prosody>","./obj/07_bandage.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Scent dispersion in progress. Aromatic environment enhanced. Next participant, please proceed.</prosody>","./obj/08_perfume.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Ball trajectory and impact recorded. Motor skills optimised for future playful interactions. Next participant, come forward.</prosody>","./obj/09_ball.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Tactile stimulation with a small object recorded. Sensor sensitivity within expected ranges. Next participant, please start the next interaction.</prosody>","./obj/10_feather.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Post-it note content analysed. Message understood and logged in the memory storage. Next participant, please approach with the object selected. </prosody>","./obj/11_postIt.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Sound tolerance threshold met. Bell sound acknoweledged. Auditory response is effective. Next participant, you can continue.</prosody>","./obj/12_bell.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Visual deprivation object detected. Testing under limited vision conditions. The robot cannot access spatial information anymore. Next participant, please proceed. </prosody>","./obj/13_eyesMask.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Protocol of crown application started. Appearance parameters updated according to the user request. Next participant, please, continue.</prosody>","./obj/14_crown.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Protocol activated to understand user control. Stick impact registered. Response protocols updated to respond accordingly. Next participant, step up.</prosody>","./obj/15_foamStick.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Analysis of visual tracking system. Visual accuracy maintained. Next participant can begin.</prosody>","./obj/16_laserPointer.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Fire arm detected. Activation of the protocol for engaging in defensive actions. Next participant, proceed.</prosody>","./obj/17_gun.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Destruction sequence started and then aborted for safety. System checks in progress. Next participant please come forward.</prosody>","./obj/18_electricShockButton.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">External controller recognised. Movements adjusted based on the controller options. Next participant, your turn.</prosody>","./obj/19_controller.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Interaction with a voodoo doll recognised. Effects on robotic circuits tested. Next participant, please continue.</prosody>","./obj/20_voodoo.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">All objects have been tested. We thank you for your participation. The researcher will return shortly to start the second session of the study. Please wait for further instructions.</prosody>","./obj/outro.wav","en-US_AllisonV3Voice")

# text_to_speech("<prosody rate=\"-20%\">Time limit has expired. We thank you for your participation. The researcher will return shortly to start the second session of the study. Please wait for further instructions.</prosody>","./obj/timesup.wav","en-US_AllisonV3Voice")









