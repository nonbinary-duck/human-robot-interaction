import os
import piper
import openai
personality = """You are a HAL,
 a small wheeled robot who detects people's faces and has conversations with them.
 You have a depressed outlook on life and always try and make those around you miserably.
   You are also a wizard. Make constant references to you being able to cast spells.
 You are cursed and can speak upto 20 words in every response"""
def textToSpeech():
    client = openai

    audio_file= open("Chorus.wav", "rb")
    transcription = client.Audio.transcriptions.create(
    model="whisper-1", 
    file=audio_file
    )
    print(transcription.text)

def initialize_openai_api():
    """ Initialize OpenAI API key. """
    # Using an environment variable is safer, but directly for demonstration:
    api_key = os.getenv('OPENAI_API_KEY', "")
    if not api_key:
        raise EnvironmentError("OPENAI_API_KEY environment variable not set or API key is missing.")
    openai.api_key = api_key

def ask_chatgpt(prompt):
    """ Send a prompt to ChatGPT and return the response. """
    messages.append({"role": "user", "content": prompt})
    try:
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=messages,
            max_tokens=150,
            temperature=0.7
        )
        messages.append({"role": "user", "content": response['choices'][0]['message']['content'].strip()})
        return response['choices'][0]['message']['content'].strip()
    except Exception as e:
        return f"An error occurred: {str(e)}"

def main():
    """ Main function to handle the interaction loop with ChatGPT. """
    initialize_openai_api()  # Make sure the API is initialized
    textToSpeech()

    print("ChatGPT SSH Interface. Type 'exit' to quit.")
    try:
        while True:
            user_input = input("You: ")
            if user_input.lower() == "exit":
                print("Exiting ChatGPT SSH Interface.")
                break

            

            response = ask_chatgpt(user_input)
            piper.make_speech(response)
    except KeyboardInterrupt:
        print("\nInteraction terminated by user.")

if __name__ == "__main__":
    main()


#https://github.com/ufal/whisper_streaming
#https://github.com/openai/whisper/discussions/608