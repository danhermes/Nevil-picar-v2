#!/bin/bash

echo "? Killing audio demons and reclaiming HiFiBerry..."

# Stop user-level pipewire & pulseaudio services
systemctl --user stop pipewire pipewire-pulse wireplumber pulseaudio 2>/dev/null

# Disable them permanently
systemctl --user disable pipewire pipewire-pulse wireplumber pulseaudio 2>/dev/null

# Kill any surviving processes
killall -9 pipewire wireplumber pulseaudio 2>/dev/null

# Remove autostart files
rm -f ~/.config/autostart/pipewire.desktop
rm -f ~/.config/autostart/wireplumber.desktop
rm -f ~/.config/autostart/pulseaudio.desktop

# Uninstall everything audio-servery
sudo apt purge -y pipewire wireplumber pulseaudio pipewire-audio-client-libraries

# Autoremove leftovers
sudo apt autoremove -y

# Unmask and re-enable ALSA utils
sudo systemctl unmask alsa-utils.service
sudo systemctl enable alsa-utils.service

# Write /etc/asound.conf to use HiFiBerry (card 3) with dmix and softvol
sudo tee /etc/asound.conf > /dev/null <<EOF
pcm.hifiberry {
    type hw
    card 3
}

ctl.hifiberry {
    type hw
    card 3
}

pcm.dmixer {
    type dmix
    ipc_key 1024
    slave {
        pcm "hifiberry"
        rate 44100
        channels 2
    }
}

ctl.dmixer {
    type hw
    card 3
}

pcm.softvol {
    type softvol
    slave.pcm "dmixer"
    control.name "PCM"
    control.card 3
}

ctl.softvol {
    type hw
    card 3
}

pcm.!default {
    type plug
    slave.pcm "softvol"
}
EOF

echo "? PipeWire purged, ALSA back in control, HiFiBerry set as default."
echo "? Rebooting to finalize changes..."

sudo reboot
