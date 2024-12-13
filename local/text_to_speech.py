import json
import re
import os
from datetime import datetime
from ibm_watson import TextToSpeechV1
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator

def count_spoken_characters(text):
    """Count the characters in the text after removing SSML tags."""
    clean_text = re.sub(r"<[^>]+>", "", text)
    return len(clean_text)

def update_character_count(characters_used):
    """Update the character count for the current month."""
    count_file = "character_count.json"
    now = datetime.now()
    current_month = now.strftime("%Y-%m")  # Format: YYYY-MM
    
    # Load the existing data or initialize
    if os.path.exists(count_file):
        with open(count_file, "r") as file:
            data = json.load(file)
    else:
        data = {"month": current_month, "count": 0}
    
    # Reset count if the month has changed
    if data["month"] != current_month:
        data = {"month": current_month, "count": 0}
    
    # Update the count
    data["count"] += characters_used
    
    # Save back to the file
    with open(count_file, "w") as file:
        json.dump(data, file)
    
    return data["count"]

def text_to_speech(text, output_filename='output.wav', voice='en-GB_CharlotteV3Voice'):

    characters_used = count_spoken_characters(text)
    total_characters = update_character_count(characters_used)

    # Load the API key and URL from the 'api.key' JSON file
    with open("api.key", "r") as file:
        config = json.load(file)
        api_key = config["api_key"]
        url = config["url"]

    # Set up the IBM Watson Text-to-Speech service
    authenticator = IAMAuthenticator(api_key)
    text_to_speech_service = TextToSpeechV1(authenticator=authenticator)
    text_to_speech_service.set_service_url(url)

    # Convert text to speech and save it as a WAV file
    with open(output_filename, 'wb') as audio_file:
        response = text_to_speech_service.synthesize(
            text,
            voice=voice,  # You can choose other voices available in IBM Watson
            accept='audio/wav'
        ).get_result()
        audio_file.write(response.content)
    print(f"Audio content saved to {output_filename}")
    print(f"Characters used this month: {total_characters}")