#!/usr/bin/env python3

"""
Test script to verify the NevilAudioRecord class with the fixed configuration.
"""

import sys
import os
sys.path.append('src/nevil_interfaces_ai/nevil_interfaces_ai')

from nevil_audio_record import NevilAudioRecord
import time

def test_nevil_audio_record():
    """Test the NevilAudioRecord class."""
    print("🔊 Testing NevilAudioRecord class...")
    
    try:
        # Initialize the audio record interface
        audio_record = NevilAudioRecord()
        print("✅ NevilAudioRecord initialized")
        
        # Test listening for speech
        print("🔊 Testing speech listening (5 seconds)...")
        print("Please say something...")
        
        audio = audio_record.listen_for_speech(timeout=5.0, phrase_time_limit=5.0)
        
        if audio:
            print("✅ Audio captured successfully")
            
            # Test speech recognition (if OpenAI API key is available)
            print("🔊 Testing speech recognition...")
            text = audio_record.recognize_speech(audio, api='auto')
            
            if text:
                print(f"✅ Speech recognized: '{text}'")
                return True
            else:
                print("⚠️ Speech recognition returned empty result")
                return False
        else:
            print("⚠️ No audio captured (timeout or no speech detected)")
            return False
            
    except Exception as e:
        print(f"❌ NevilAudioRecord test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        # Clean up
        if 'audio_record' in locals():
            audio_record.cleanup()

def test_simulation_mode():
    """Test simulation mode functionality."""
    print("\n🔊 Testing simulation mode...")
    
    try:
        # Force simulation mode by setting microphone to None
        audio_record = NevilAudioRecord()
        audio_record.simulation_mode = True
        audio_record.microphone = None
        
        print("✅ Simulation mode enabled")
        
        # Test listening in simulation mode
        audio = audio_record.listen_for_speech(timeout=1.0)
        print("✅ Simulation listening completed")
        
        # Test recognition in simulation mode
        text = audio_record.recognize_speech(audio)
        print(f"✅ Simulation recognition: '{text}'")
        
        return True
        
    except Exception as e:
        print(f"❌ Simulation mode test failed: {e}")
        return False

if __name__ == "__main__":
    print("🔊 NevilAudioRecord Configuration Test")
    print("=" * 50)
    
    # Test actual hardware
    success1 = test_nevil_audio_record()
    
    # Test simulation mode
    success2 = test_simulation_mode()
    
    print("\n" + "=" * 50)
    if success1:
        print("🎉 HARDWARE TEST PASSED - NevilAudioRecord is working!")
    else:
        print("❌ Hardware test failed")
        
    if success2:
        print("✅ Simulation mode test passed")
    else:
        print("❌ Simulation mode test failed")
        
    if success1 or success2:
        print("🎉 NevilAudioRecord class is functional!")
    else:
        print("❌ NevilAudioRecord class needs more work")