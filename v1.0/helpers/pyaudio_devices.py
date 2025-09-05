import pyaudio._portaudio as p

#p = pyaudio.PyAudio()
p.initialize()
print(f"Devices: {p.get_device_count()}")
# for i in range(p.get_device_count()):
#     info = p.get_device_info_by_index(i)
#     print(f"Device {i}: {info}")
#     if info['maxInputChannels'] > 0:
#         print(f"{i}: {info['name']} - {info['maxInputChannels']} channels")
