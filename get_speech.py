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


text_to_speech("<express-as style=\"cheerful\">Hi! Welcome, everyone! Thank you for coming here. Please come forward. We'll start soon!</express-as>","intro.wav","en-US_EmmaExpressive")
