#!/bin/bash

# Fix ALSA underrun errors in Nevil-picar v2.0
# This script applies configuration changes to prevent audio buffer underruns

echo "🔧 Fixing ALSA underrun errors..."

# Copy ALSA configuration to system location
echo "📁 Installing ALSA configuration..."
sudo cp .asoundrc /etc/asound.conf
cp .asoundrc ~/.asoundrc

# Set ALSA environment variables
echo "🌍 Setting ALSA environment variables..."
export ALSA_BUFFER_SIZE=4096
export ALSA_PERIOD_SIZE=1024
export PULSE_LATENCY_MSEC=60

# Add to bashrc for persistence
echo "💾 Making environment variables persistent..."
echo "# ALSA optimization for Nevil-picar v2.0" >> ~/.bashrc
echo "export ALSA_BUFFER_SIZE=4096" >> ~/.bashrc
echo "export ALSA_PERIOD_SIZE=1024" >> ~/.bashrc
echo "export PULSE_LATENCY_MSEC=60" >> ~/.bashrc
echo "export AUDIO_SAMPLE_RATE=44100" >> ~/.bashrc
echo "export AUDIO_CHANNELS=2" >> ~/.bashrc
echo "export AUDIO_CHUNK_SIZE=2048" >> ~/.bashrc

# Restart ALSA and PulseAudio services
echo "🔄 Restarting audio services..."
sudo systemctl restart alsa-state || echo "ALSA state service not found"
pulseaudio --kill 2>/dev/null || echo "PulseAudio not running"
pulseaudio --start 2>/dev/null || echo "Could not start PulseAudio"

# Build the updated audio interface
echo "🔨 Building updated audio interface..."
cd src/nevil_interfaces_ai
python3 -m pip install -e . --user
cd ../..

# Build the entire project
echo "🏗️ Building Nevil project..."
colcon build --packages-select nevil_interfaces_ai

echo "✅ ALSA underrun fixes applied successfully!"
echo ""
echo "🎯 Changes made:"
echo "   • Increased audio buffer sizes (4096 samples)"
echo "   • Increased period sizes (1024 samples)"
echo "   • Set sample rate to 44100 Hz"
echo "   • Configured stereo audio (2 channels)"
echo "   • Added ALSA configuration file"
echo "   • Set PulseAudio latency to 60ms"
echo ""
echo "🔄 Please restart your ROS2 nodes to apply the changes:"
echo "   ./ros2_start.sh"
echo ""
echo "📊 Monitor for improvements in audio quality and reduced underrun errors."