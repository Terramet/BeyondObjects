import json
from ibm_watson import TextToSpeechV1
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator
import os

def text_to_speech(text, output_filename='output.wav', voice='en-GB_CharlotteV3Voice'):
    # Load the API key and URL from the 'api.key' JSON file
    with open("api.key", "r") as file:
        config = json.load(file)
        api_key = config["api_key"]
        url = config["url"]

    output_path = os.path.join('./output', output_filename)
    # Set up the IBM Watson Text-to-Speech service
    authenticator = IAMAuthenticator(api_key)
    text_to_speech_service = TextToSpeechV1(authenticator=authenticator)
    text_to_speech_service.set_service_url(url)

    # Convert text to speech and save it as a WAV file
    with open(output_path, 'wb') as audio_file:
        response = text_to_speech_service.synthesize(
            text,
            voice=voice,  # You can choose other voices available in IBM Watson
            accept='audio/wav'
        ).get_result()
        audio_file.write(response.content)
    print(f"Audio content saved to {output_path}")