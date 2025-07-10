# ALSA Underrun Fixes for Nevil-picar v2.0

## Problem Description

The speech recognition and synthesis nodes were experiencing ALSA underrun errors:
```
ALSA lib pcm.c:8570:(snd_pcm_recover) underrun occurred
```

These errors indicate that the audio buffer is running out of data, causing audio dropouts and poor performance.

## Root Causes

1. **Small buffer sizes** - Default pygame mixer buffer (512 samples) was too small
2. **Low sample rate** - 16kHz sample rate caused compatibility issues
3. **Mono audio** - Single channel audio didn't match hardware expectations
4. **Missing ALSA configuration** - No system-level buffer management
5. **Inadequate environment variables** - No ALSA-specific optimizations

## Implemented Fixes

### 1. Audio Parameter Optimization

**File:** `src/nevil_interfaces_ai/nevil_interfaces_ai/audio_hardware_interface.py`

- **Sample Rate:** Increased from 16kHz to 44.1kHz for better ALSA compatibility
- **Channels:** Changed from mono (1) to stereo (2) for hardware compatibility
- **Chunk Size:** Increased from 1024 to 2048 samples
- **ALSA Buffer:** Set to 4096 samples (larger buffer prevents underruns)
- **ALSA Period:** Set to 1024 samples (optimal period size)

### 2. ALSA Environment Configuration

Added `_configure_alsa_environment()` method that sets:
- `ALSA_BUFFER_SIZE=4096`
- `ALSA_PERIOD_SIZE=1024`
- `PULSE_LATENCY_MSEC=60`

### 3. Pygame Mixer Optimization

Updated pygame mixer initialization to use:
```python
pygame.mixer.init(
    frequency=self.sample_rate,  # 44100 Hz
    size=-16,                    # 16-bit signed
    channels=self.channels,      # 2 (stereo)
    buffer=self.chunk_size       # 2048 samples
)
```

### 4. System ALSA Configuration

**File:** `.asoundrc`

Created comprehensive ALSA configuration with:
- Hardware PCM with optimized buffer sizes
- Software mixing (dmix) with larger buffers
- Capture device (dsnoop) optimization
- Full duplex configuration

### 5. Environment Variables

Added persistent environment variables:
- `AUDIO_SAMPLE_RATE=44100`
- `AUDIO_CHANNELS=2`
- `AUDIO_CHUNK_SIZE=2048`
- `ALSA_BUFFER_SIZE=4096`
- `ALSA_PERIOD_SIZE=1024`
- `PULSE_LATENCY_MSEC=60`

## Configuration Details

### Buffer Size Calculations

- **Buffer Size (4096 samples):** At 44.1kHz = ~93ms of audio data
- **Period Size (1024 samples):** At 44.1kHz = ~23ms periods
- **Chunk Size (2048 samples):** At 44.1kHz = ~46ms processing chunks

These larger buffers provide sufficient headroom to prevent underruns while maintaining acceptable latency.

### Sample Rate Selection

44.1kHz was chosen because:
- Standard audio CD quality
- Well-supported by most audio hardware
- Good balance between quality and processing overhead
- Compatible with most ALSA drivers

## Usage

### Automatic Application

Run the fix script:
```bash
./fix_alsa_underruns.sh
```

### Manual Application

1. Copy ALSA configuration:
```bash
sudo cp .asoundrc /etc/asound.conf
cp .asoundrc ~/.asoundrc
```

2. Set environment variables:
```bash
export ALSA_BUFFER_SIZE=4096
export ALSA_PERIOD_SIZE=1024
export PULSE_LATENCY_MSEC=60
```

3. Rebuild the audio interface:
```bash
colcon build --packages-select nevil_interfaces_ai
```

4. Restart ROS2 nodes:
```bash
./ros2_start.sh
```

## Verification

After applying the fixes, monitor the logs for:

### Success Indicators
- No more "underrun occurred" messages
- Smooth audio playback without dropouts
- Consistent speech recognition performance
- Log message: "ALSA environment configured: buffer_size=4096, period_size=1024"

### Performance Monitoring
```bash
# Monitor ALSA status
cat /proc/asound/cards

# Check audio device info
aplay -l

# Test audio playback
speaker-test -t wav -c 2
```

## Troubleshooting

### If underruns persist:

1. **Increase buffer sizes further:**
```bash
export ALSA_BUFFER_SIZE=8192
export ALSA_PERIOD_SIZE=2048
```

2. **Check audio hardware:**
```bash
lsusb | grep -i audio
dmesg | grep -i audio
```

3. **Verify ALSA configuration:**
```bash
aplay -D hw:0,0 /usr/share/sounds/alsa/Front_Left.wav
```

4. **Monitor system load:**
```bash
top -p $(pgrep -f "speech_recognition\|speech_synthesis")
```

## Technical Notes

### Why These Settings Work

1. **Larger buffers** provide more time for the system to refill audio data
2. **Higher sample rate** reduces quantization noise and improves compatibility
3. **Stereo audio** matches most hardware expectations
4. **Optimized periods** balance latency with stability
5. **Environment variables** ensure consistent configuration across all audio components

### Performance Impact

- **Memory usage:** Slightly increased due to larger buffers (~32KB additional)
- **CPU usage:** Minimal increase due to higher sample rate
- **Latency:** Acceptable increase (~93ms total latency)
- **Quality:** Significantly improved audio stability

## Related Files

- `src/nevil_interfaces_ai/nevil_interfaces_ai/audio_hardware_interface.py` - Main audio interface
- `.asoundrc` - ALSA configuration
- `fix_alsa_underruns.sh` - Automated fix script
- `docs/audio ALSA/ALSA_UNDERRUN_FIXES.md` - This documentation

## Version History

- **v2.0.1** - Initial ALSA underrun fixes implemented
- **Date:** January 9, 2025
- **Author:** Nevil-picar v2.0 Development Team