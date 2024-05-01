import random
import subprocess

def make_speech(text: str):
    # Generate some speech and play it using aplay with our bash script
    subprocess.call(f"./tts.sh \"{text}\" {random.randint(0, 108)}", shell=True);
    
