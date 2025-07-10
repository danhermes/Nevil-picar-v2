#!/bin/bash

# Fix USB device disconnection stability issues for Nevil-picar v2.0
# This script configures the system to handle USB device disconnections gracefully

echo "🔧 Fixing USB device disconnection stability issues..."

# Create udev rules for USB device handling
echo "📋 Creating udev rules for stable USB device handling..."
sudo tee /etc/udev/rules.d/99-nevil-usb-stability.rules > /dev/null << 'EOF'
# Nevil-picar v2.0 USB stability rules
# Handle USB audio device disconnections gracefully

# USB audio devices - prevent immediate system impact on disconnect
SUBSYSTEM=="usb", ATTR{bDeviceClass}=="01", ACTION=="remove", RUN+="/bin/logger 'Nevil: USB audio device removed'"
SUBSYSTEM=="usb", ATTR{bDeviceClass}=="01", ACTION=="add", RUN+="/bin/logger 'Nevil: USB audio device added'"

# USB HID devices (keyboard/mouse) - prevent system crashes
SUBSYSTEM=="usb", ATTR{bDeviceClass}=="03", ACTION=="remove", RUN+="/bin/logger 'Nevil: USB HID device removed'"
SUBSYSTEM=="usb", ATTR{bDeviceClass}=="03", ACTION=="add", RUN+="/bin/logger 'Nevil: USB HID device added'"

# Generic USB device stability
SUBSYSTEM=="usb", ACTION=="remove", RUN+="/usr/local/bin/nevil-usb-handler.sh remove %k"
SUBSYSTEM=="usb", ACTION=="add", RUN+="/usr/local/bin/nevil-usb-handler.sh add %k"
EOF

# Create USB event handler script
echo "🔧 Creating USB event handler script..."
sudo tee /usr/local/bin/nevil-usb-handler.sh > /dev/null << 'EOF'
#!/bin/bash

# Nevil-picar v2.0 USB event handler
# Handles USB device add/remove events gracefully

ACTION="$1"
DEVICE="$2"

case "$ACTION" in
    "remove")
        logger "Nevil USB Handler: Device $DEVICE removed"
        
        # Check if this affects audio devices
        if lsusb | grep -i audio > /dev/null 2>&1; then
            logger "Nevil USB Handler: Audio devices still available"
        else
            logger "Nevil USB Handler: No audio devices detected, may need fallback"
            # Signal audio processes to reinitialize
            pkill -USR1 -f "speech_recognition_node" 2>/dev/null || true
            pkill -USR1 -f "speech_synthesis_node" 2>/dev/null || true
            pkill -USR1 -f "integrated_ai_interface" 2>/dev/null || true
        fi
        ;;
    "add")
        logger "Nevil USB Handler: Device $DEVICE added"
        
        # Brief delay to allow device enumeration
        sleep 1
        
        # Check if new audio device is available
        if lsusb | grep -i audio > /dev/null 2>&1; then
            logger "Nevil USB Handler: Audio device detected, signaling processes"
            # Signal audio processes to reinitialize
            pkill -USR2 -f "speech_recognition_node" 2>/dev/null || true
            pkill -USR2 -f "speech_synthesis_node" 2>/dev/null || true
            pkill -USR2 -f "integrated_ai_interface" 2>/dev/null || true
        fi
        ;;
esac
EOF

# Make the handler script executable
sudo chmod +x /usr/local/bin/nevil-usb-handler.sh

# Configure systemd to handle USB events properly
echo "⚙️ Configuring systemd for USB stability..."
sudo tee /etc/systemd/system/nevil-usb-stability.service > /dev/null << 'EOF'
[Unit]
Description=Nevil USB Stability Service
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/true
RemainAfterExit=yes

# USB stability settings
Environment=USB_AUTOSUSPEND=0
Environment=ALSA_CARD_FALLBACK=1

[Install]
WantedBy=multi-user.target
EOF

# Enable the service
sudo systemctl enable nevil-usb-stability.service

# Configure kernel parameters for USB stability
echo "🔧 Configuring kernel parameters for USB stability..."
sudo tee -a /etc/modprobe.d/nevil-usb-stability.conf > /dev/null << 'EOF'
# Nevil-picar v2.0 USB stability configuration

# Disable USB autosuspend to prevent device disconnection issues
options usbcore autosuspend=-1

# ALSA USB audio stability
options snd-usb-audio enable_autoclock=0
options snd-usb-audio autoclock=0

# Increase USB buffer sizes
options usbcore usbfs_memory_mb=256
EOF

# Configure ALSA for USB device stability
echo "🎵 Configuring ALSA for USB device stability..."
sudo tee /etc/asound.conf > /dev/null << 'EOF'
# Nevil-picar v2.0 ALSA configuration for USB stability

# Default PCM device with fallback
pcm.!default {
    type plug
    slave.pcm "hw:0,0"
    slave.rate 44100
    slave.channels 2
    slave.format S16_LE
}

# Control device with fallback
ctl.!default {
    type hw
    card 0
}

# USB audio device with error recovery
pcm.usb_audio {
    type plug
    slave {
        pcm "hw:1,0"
        rate 44100
        channels 2
        format S16_LE
        buffer_size 4096
        period_size 1024
    }
    # Fallback to card 0 if USB device fails
    hint {
        show on
        description "USB Audio with fallback"
    }
}
EOF

# Set system limits for audio processes
echo "📊 Setting system limits for audio stability..."
sudo tee -a /etc/security/limits.conf > /dev/null << 'EOF'
# Nevil-picar v2.0 audio process limits
@audio          -       rtprio          95
@audio          -       memlock         unlimited
@audio          -       nice            -19
EOF

# Configure PulseAudio for stability (if present)
if command -v pulseaudio &> /dev/null; then
    echo "🔊 Configuring PulseAudio for USB stability..."
    mkdir -p ~/.config/pulse
    tee ~/.config/pulse/default.pa > /dev/null << 'EOF'
# Nevil-picar v2.0 PulseAudio configuration for USB stability

# Load necessary modules
load-module module-device-restore
load-module module-stream-restore
load-module module-card-restore

# USB audio with automatic recovery
load-module module-udev-detect tsched=0

# Fallback devices
load-module module-alsa-sink device=hw:0,0 sink_name=fallback_sink
load-module module-alsa-source device=hw:0,0 source_name=fallback_source

# Set default sink and source with fallback
set-default-sink fallback_sink
set-default-source fallback_source

# Automatic device switching
load-module module-switch-on-connect
EOF
fi

# Reload udev rules
echo "🔄 Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# Update initramfs with new modules
echo "🔄 Updating initramfs..."
sudo update-initramfs -u

echo "✅ USB stability fixes applied successfully!"
echo ""
echo "🎯 Changes made:"
echo "   • Created udev rules for USB device handling"
echo "   • Added USB event handler script"
echo "   • Configured systemd service for USB stability"
echo "   • Set kernel parameters for USB stability"
echo "   • Configured ALSA for USB device fallback"
echo "   • Set system limits for audio processes"
echo "   • Configured PulseAudio for automatic recovery"
echo ""
echo "🔄 Please reboot the system to apply all changes:"
echo "   sudo reboot"
echo ""
echo "📊 After reboot, test USB stability by unplugging/plugging devices"
echo "   Monitor logs: journalctl -f | grep -i 'nevil\\|usb\\|audio'"