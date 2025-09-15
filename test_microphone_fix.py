#!/usr/bin/env python3

"""
Test script to verify microphone access with the fixed configuration.
"""

import speech_recognition as sr
import time

def test_microphone_access():
    """Test microphone access with device index 1."""
    print("🔊 Testing microphone access with device index 1...")
    
    try:
        # Initialize recognizer
        recognizer = sr.Recognizer()
        print("✅ Speech recognizer initialized")
        
        # Initialize microphone with device index 1
        microphone = sr.Microphone(device_index=1, sample_rate=44100, chunk_size=1024)
        print("✅ Microphone initialized with device index 1")
        
        # Test microphone context manager
        with microphone as source:
            print("✅ Microphone context manager working")
            print("🔊 Adjusting for ambient noise...")
            recognizer.adjust_for_ambient_noise(source, duration=1)
            print("✅ Ambient noise adjustment completed")
            
            print("🔊 Listening for 3 seconds...")
            audio = recognizer.listen(source, timeout=3, phrase_time_limit=3)
            print("✅ Audio captured successfully")
            
        print("🎉 All microphone tests passed!")
        return True
        
    except Exception as e:
        print(f"❌ Microphone test failed: {e}")
        return False

def test_default_microphone():
    """Test default microphone access."""
    print("\n🔊 Testing default microphone access...")
    
    try:
        # Initialize recognizer
        recognizer = sr.Recognizer()
        
        # Initialize microphone with default device
        microphone = sr.Microphone()
        print("✅ Default microphone initialized")
        
        # Test microphone context manager
        with microphone as source:
            print("✅ Default microphone context manager working")
            print("🔊 Quick test listening for 1 second...")
            audio = recognizer.listen(source, timeout=1, phrase_time_limit=1)
            print("✅ Default microphone audio captured")
            
        return True
        
    except Exception as e:
        print(f"❌ Default microphone test failed: {e}")
        return False

if __name__ == "__main__":
    print("🔊 Microphone Configuration Test")
    print("=" * 40)
    
    # Test specific device index 1
    success1 = test_microphone_access()
    
    # Test default device
    success2 = test_default_microphone()
    
    print("\n" + "=" * 40)
    if success1 and success2:
        print("🎉 ALL TESTS PASSED - Microphone configuration is working!")
    elif success1:
        print("✅ Device index 1 works, default device has issues")
    elif success2:
        print("✅ Default device works, device index 1 has issues")
    else:
        print("❌ Both tests failed - microphone configuration needs more work")