import os
import sys
import numpy as np
import time

GRAY = '1;30'
RED = '0;31'
GREEN = '0;32'
YELLOW = '0;33'
BLUE = '0;34'
PURPLE = '0;35'
DARK_GREEN = '0;36'
WHITE = '0;37'

def print_color(msg, end='\n', file=sys.stdout, flush=False, color=''):
    print('\033[%sm%s\033[0m'%(color, msg), end=end, file=file, flush=flush)

def gray_print(msg, end='\n', file=sys.stdout, flush=False):
    print_color(msg, end=end, file=file, flush=flush, color=GRAY)

def warn(msg, end='\n', file=sys.stdout, flush=False):
    print_color(msg, end=end, file=file, flush=flush, color=YELLOW)

def error(msg, end='\n', file=sys.stdout, flush=False):
    print_color(msg, end=end, file=file, flush=flush, color=RED)

def redirect_error_2_null():
    # https://github.com/spatialaudio/python-sounddevice/issues/11

    devnull = os.open(os.devnull, os.O_WRONLY)
    old_stderr = os.dup(2)
    sys.stderr.flush()
    os.dup2(devnull, 2)
    os.close(devnull)
    return old_stderr

def cancel_redirect_error(old_stderr):
    os.dup2(old_stderr, 2)
    os.close(old_stderr)

def run_command(cmd):
    """
    Run command and return status and output

    :param cmd: command to run
    :type cmd: str
    :return: status, output
    :rtype: tuple
    """
    import subprocess
    p = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    result = p.stdout.read().decode('utf-8')
    status = p.poll()
    return status, result

def sox_volume(input_file, output_file, volume):
    import sox

    try:
        transform = sox.Transformer()
        transform.vol(volume)

        transform.build(input_file, output_file)

        return True
    except Exception as e:
        print(f"sox_volume err: {e}")
        return False


speak_first = False

def speak_block(music, name, volume=100):
    """
    speak, play audio with block

    :param name: the file name int the folder(SOUND_DIR)
    :type name: str
    :param volume: volume, 0-100
    :type volume: int
    """
    global speak_first
    is_run_with_root = (os.geteuid() == 0)
    if not is_run_with_root and not speak_first:
        speak_first = True
        warn("Play sound needs to be run with sudo.")
    _status, _ = run_command('sudo killall pulseaudio') # Solve the problem that there is no sound when running in the vnc environment
    
    if os.path.isfile(name):
        music.sound_play(name, volume)
    else:
        warn(f'No sound found for {name}')
        return False

def check_audio_energy(audio_data, threshold=0.009):
    """Check if audio has enough energy to process"""
    # Convert audio to numpy array
    audio_array = np.frombuffer(audio_data.get_raw_data(), dtype=np.int16)
    normalized = audio_array.astype(np.float32) / 32768.0
    rms_energy = np.sqrt(np.mean(np.square(normalized)))
    
    print("RMS Energy:", rms_energy)
    return rms_energy >= threshold

def generate_tts_filename(prefix="", volume_db=None):
    """Generate timestamped filenames for TTS files"""
    timestamp = time.strftime("%y-%m-%d_%H-%M-%S", time.localtime())
    raw_file = f"./tts/{prefix}{timestamp}_raw.wav"
    
    if volume_db is not None:
        processed_file = f"./tts/{prefix}{timestamp}_{volume_db}dB.wav"
        return raw_file, processed_file
    return raw_file

def play_audio_file(music, tts_file):
    """Play audio file and clean up after"""
    try:
        if os.path.exists(tts_file):
            music.music_play(tts_file)
            while music.pygame.mixer.music.get_busy():
                time.sleep(0.1)
            music.music_stop()
            os.remove(tts_file)
    except Exception as e:
        print(f"Error playing TTS: {e}")

def play_audio_data(music, audio_data, volume_db):
    """Play AudioData object and clean up temp files"""
    # Convert AudioData to wav file format that pygame can play
    temp_wav = "./tts/temp_recording.wav"
    with open(temp_wav, "wb") as f:
        f.write(audio_data.get_wav_data())  # Convert AudioData to WAV format
    temp_wav_vol = "./tts/temp_recording_vol.wav"
    status = sox_volume(temp_wav, temp_wav_vol, volume_db)
    
    # Play the temporary wav file
    music.music_play(temp_wav_vol)
    while music.pygame.mixer.music.get_busy():
        time.sleep(0.1)
    music.music_stop()
    
    # Clean up temporary files
    try:
        os.remove(temp_wav)
        os.remove(temp_wav_vol)
    except:
        pass

def wait_for_completion(obj):
    """Wait for speech and actions to complete"""
    wait_start = time.time()
    
    if obj.speech_loaded:
        while True:
            with obj.speech_lock:
                if not obj.speech_loaded:
                    break
            time.sleep(.01)

    while True:
        with obj.action_lock:
            if obj.action_status != 'actions':
                break
        time.sleep(.01)
    
    gray_print(f"Wait time for completion: {time.time() - wait_start:.3f}s")
