
# Nevil AI Architecture and Autonomous Mode

## 1. System Architecture Summary

**Platform:** Raspberry Pi (SunFounder PiCar-X)  
**Purpose:** Conversational assistant robot with expressive physical and vocal behaviors

### Core Capabilities

| Function        | Technology                     |
|----------------|---------------------------------|
| Speech Input    | OpenAI Whisper + mic (alsamixer) |
| Speech Output   | OpenAI TTS                     |
| Conversation    | OpenAI Assistant API (with mood + behavior logic) |
| Motion Control  | PiCar-X DC motors + servos     |
| Camera Control  | 2-axis gimbal-mounted camera   |
| Action Engine   | JSON-style action dispatcher   |

---

## 2. Mood System

Nevil operates based on a **mood profile**, which contains the following traits:

| Trait        | Description                                        |
|--------------|----------------------------------------------------|
| `volume`     | Affects sound output, honking, and expressiveness |
| `curiosity`  | Affects desire to explore or observe              |
| `sociability`| Affects likelihood to engage others               |
| `whimsy`     | Affects likelihood of playful, non-functional acts|
| `energy`     | Affects motion strength, duration, pacing         |

### Example Moods

- `playful`: High energy, high whimsy
- `brooding`: Low sociability, low energy
- `curious`: High curiosity
- `melancholic`: Low across all traits
- `zippy`: High energy and motion-oriented
- `lonely`: High sociability but subdued otherwise
- `mischievous`: High whimsy, medium sociability
- `sleepy`: Low energy, quiet

---

## 3. Behavior Weighting and Selection

### Base Behavior Weights

Each behavior has a default probability of occurring when idle:

```python
BEHAVIOR_BASE_WEIGHTS = {
    "explore": 0.3,
    "rest": 0.1,
    "sleep": 0.1,
    "fidget": 0.0,
    "address": 0.0,
    "play": 0.2,
    "panic": 0.05,
    "circle": 0.05,
    "sing": 0.05,
    "mutter": 0.05,
    "dance": 0.1
}
```

### Behavior Trait Biases

Behaviors are influenced by traits:

```python
BEHAVIOR_TRAIT_BIASES = {
    "explore":     {"curiosity": 1.2, "energy": 1.1},
    "rest":        {"energy": 0.6},
    "sleep":       {"energy": 0.3},
    "fidget":      {"whimsy": 1.2, "energy": 0.8},
    "address":     {"sociability": 1.4},
    "play":        {"whimsy": 1.2, "energy": 1.2},
    "panic":       {"energy": 1.5},
    "circle":      {"curiosity": 1.1, "energy": 1.1},
    "sing":        {"whimsy": 1.3},
    "mutter":      {"curiosity": 0.7, "sociability": 0.6},
    "dance":       {"energy": 1.3, "whimsy": 1.5}
}
```

---

## 4. Auto Mode Loop

Nevil runs a loop during idle time:

```python
class AutoModule:
    def __init__(self):
        self.current_mood_name = "curious"
        self.current_mood = MOOD_PROFILES[self.current_mood_name]

    def set_mood(self, mood_name=None, trait_overrides=None):
        ...

    def run_idle_loop(self, cycles=3):
        for _ in range(cycles):
            weights = compute_behavior_weights(...)
            behavior = weighted_choice(weights)
            BEHAVIOR_FUNCTIONS[behavior](self.current_mood)

    def get_mood_param_str(self):
        return f"mood={...} volume={...} curiosity={...}"
```

- Behaviors adapt based on mood traits.
- Actions are expressive but aligned to their core purpose.
- Output is consistent, modular, and extendable.

---

## 5. Design Philosophy

- **Mood defines personality** but does not micromanage.
- **Behaviors own their identity**, not just reacting randomly.
- **Weights select**, but **traits control expression**.
- **Elegance over logic tangles.** Simplicity fuels creativity.

---

## 6. File Structure

- `automatic.py` — Full implementation
- All moods, actions, and emotional control unified in a single API
