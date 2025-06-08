# Nevil-picar v2.0: Concept Document

## Concept

Nevil-picar v2.0 is a multi-threaded, real-time robotic system built on the PiCar-X platform that integrates ROS2 with PREEMPT-RT for deterministic performance. It combines conversational AI, autonomous navigation, computer vision, and environmental mapping into a cohesive robotic companion capable of simultaneous operations across multiple processing threads.

## Goal

To create a fully functional, responsive robotic companion that can:
- Navigate autonomously while maintaining conversational interaction
- Map and learn its environment through SLAM functionality
- Recognize and track users and objects using both online and offline vision processing
- Operate in various modes (conversation, play, autonomous, sleep) with seamless transitions
- Maintain real-time responsiveness through multi-threaded execution and PREEMPT-RT integration
- Function both with a physical robot and through a digital twin simulation
- Offline and online functionality and cognition

## Problem

Current robotic systems often suffer from:
- Single-threaded execution that causes stuttering or lag during complex operations
- Inability to perform multiple functions simultaneously (e.g., movement stalls during speech processing)
- Lack of deterministic timing for critical operations like obstacle avoidance
- Limited integration between navigation, vision, and conversational capabilities
- Dependency on constant internet connectivity for AI functions
- Difficulty transitioning between simulation and physical hardware
- Implementing a sufficiently small GPT model (like tinyllama) and switching with online OPENAI model as needed
- robot motion can interfere with speech recognition (needs solution)

## Solution

Nevil-picar v2.0 addresses these challenges through:
- ROS2 architecture with MultiThreadedExecutor for true parallel processing
- PREEMPT-RT kernel integration for deterministic, low-latency performance
- Hybrid AI approach combining cloud-based (OpenAI) and local (Gemma 2/TinyLlama) models
- Modular node design with prioritized execution (critical functions get higher priority)
- SLAM implementation for environmental mapping and spatial awareness
- Digital twin simulation for testing and development without physical hardware
- Decoupled architecture allowing easy switching between simulation and physical robot
- Multi-modal interaction through text, voice, and visual interfaces
- Online GPT access (decoupled to use Gemini, OPENAI, etc.) and offline small footprint model like TinyLlama or Gemma-2 for when network isn't available or for real-time visual recognition and navigation.
- Employ powerful but delayed online GPT functionality supported by modest, sufficiently fast offline GPT model.