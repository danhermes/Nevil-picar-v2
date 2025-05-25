import random
import time
import warnings

warnings.filterwarnings("ignore", category=RuntimeWarning, module="ALSA")

# Actions: forward, backward, left, right, stop, twist left, twist right, come here, shake head, 
#    nod, wave hands, resist, act cute, rub hands, think, twist body, celebrate, depressed, keep think
#
# Sounds: honk, start engine

# -----------------------
# Main Auto Class
# -----------------------
class Automatic:
    
    # -----------------------
    # Mood Profiles (Expanded)
    # -----------------------
    MOOD_PROFILES = {
        "playful":     {"volume": 85, "curiosity": 70, "sociability": 90, "whimsy": 95, "energy": 90},
        "brooding":    {"volume": 25, "curiosity": 40, "sociability": 10, "whimsy": 15, "energy": 30},
        "curious":     {"volume": 40, "curiosity": 85, "sociability": 50, "whimsy": 35, "energy": 60},
        "melancholic": {"volume": 30, "curiosity": 30, "sociability": 20, "whimsy": 20, "energy": 20},
        "zippy":       {"volume": 70, "curiosity": 60, "sociability": 60, "whimsy": 50, "energy": 95},
        "lonely":      {"volume": 60, "curiosity": 40, "sociability": 80, "whimsy": 20, "energy": 50},
        "mischievous": {"volume": 90, "curiosity": 75, "sociability": 50, "whimsy": 95, "energy": 85},
        "sleepy":      {"volume": 10, "curiosity": 20, "sociability": 10, "whimsy": 5,  "energy": 15}
    }

    # -----------------------
    # Base Behavior Weights
    # -----------------------
    BEHAVIOR_BASE_WEIGHTS = {
        "explore": 0.3,
        "rest": 0.1,
        "sleep": 0.2,
        "fidget": 0.0,
        "address": 0.0,
        "play": 0.1,
        "panic": 0.05,
        "circle": 0.05,
        "sing": 0.05,
        "mutter": 0.05,
        "dance": 0.1
    }

    # -----------------------
    # Trait Biases for Weight Calculation
    # -----------------------
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

    # -----------------------
    # Behavior Functions
    # -----------------------
    def __init__(self, nevil_self):
        self.current_mood_name = "curious"  # Default mood
        self.current_mood = self.MOOD_PROFILES[self.current_mood_name]
        self.nevil = nevil_self  # Store the nevil reference
        self.last_interaction_time = 0  # Track when we last had an interaction
        
        # Map behavior names to their functions
        self.BEHAVIOR_FUNCTIONS = {
            "explore": self.explore,
            "rest": self.rest,
            "sleep": self.sleep,
            "fidget": self.fidget,
            "address": self.address,
            "play": self.play,
            "panic": self.panic,
            "circle": self.circle,
            "sing": self.sing,
            "mutter": self.mutter,
            "dance": self.dance
        }

    def do_actions(self, actions, mood=None):
        """Queue actions for execution by action thread"""
        print(f"[actions] {actions.all}" + (f" ({mood})" if mood else ""))
        
        # Parse to check if it's a sound
        #action, params = self.nevil.parse_action(actions)
        
        # # Queue sound effects directly to speech handler
        # if action in ["honk", "start_engine"]:
        #     with self.nevil.speech_lock:
        #         self.nevil.sound_effects_queue.append(action)
        #         self.nevil.speech_loaded = True  # Set flag so speech thread processes it
        #         self.nevil.tts_file = None  # No TTS file for sound effects
        
        # Queue action for action thread
        with self.nevil.action_lock:
            self.nevil.actions_to_be_done = [actions]
            self.nevil.action_status = 'actions'
        
        # Wait for both speech and action to complete
        while True:
            if not self.nevil.auto_enabled:
                break
            
            speech_done = True
            action_done = False
            
            with self.nevil.speech_lock:
                if self.nevil.speech_loaded:
                    speech_done = False
                
            with self.nevil.action_lock:
                if self.nevil.action_status != 'actions':
                    action_done = True
                
            if speech_done and action_done:
                break
            
            time.sleep(.01)

    def explore(self, mood):
        actions = []
        curiosity = mood["curiosity"]
        energy = mood["energy"]
        whimsy = mood["whimsy"]

        speed = int(35 + energy * 0.5)
        distance = int(50 + curiosity * 0.5)
        actions.append(f"forward {distance} {speed}")

        if curiosity > 60:
            actions.append("left")
            actions.append("sleep 0.3")
            actions.append("right")
            actions.append("sleep 0.3")

        if whimsy > 75:
            actions.append("nod")

        self.do_actions(actions, mood)

    def rest(self, mood):
        actions = []
        energy = mood["energy"]
        duration = (100 - energy) / 20
        actions.append("act cute")
        actions.append(f"sleep {duration}")
        
        self.do_actions(actions, mood)

    def sleep(self, mood):
        actions = []
        energy = mood["energy"]
        duration = (100 - energy)
        actions.append("depressed")
        actions.append("depressed")
        actions.append(f"sleep {duration}")
        
        self.do_actions(actions, mood)

    def fidget(self, mood):
        actions = []
        energy = mood["energy"]
        whimsy = mood["whimsy"]

        if energy < 30:
            actions.append("twist body")
        else:
            actions.append("shake head")

        if whimsy > 60:
            actions.append("nod")

        actions.append("sleep 0.8")

        self.do_actions(actions, mood)

    def address(self, mood):
        actions = []
        sociability = mood["sociability"]
        curiosity = mood["curiosity"]

        if sociability > 50:
            actions.append("wave hands")
            if curiosity > 40:
                actions.append("think")
            else:
                actions.append("nod")
        else:
            actions.append("resist")

        actions.append("sleep 1.0")

        self.do_actions(actions, mood)

    def play(self, mood):
        actions = []
        energy = mood["energy"]
        whimsy = mood["whimsy"]
        volume = mood["volume"]

        if volume > 50:
            actions.append("start engine")
        actions.append("left")
        
        if whimsy > 60:
            actions.append("wave hands")

        if mood["volume"] > 70:
            actions.append("honk")

        actions.append("sleep 1")

        self.do_actions(actions, mood)

    def panic(self, mood):
        actions = []
        energy = mood["energy"]
        volume = mood["volume"]

        if volume > 50:
            actions.append("start engine")
        speed = min(50, int(30 + energy * 0.9))
        actions.append(f"backward {speed} 40")

        if volume > 50:
            actions.append("honk")

        actions.append("right")
        actions.append("sleep 1")

        self.do_actions(actions, "alarmed")

    def circle(self, mood):
        actions = []
        curiosity = mood["curiosity"]
        energy = mood["energy"]

        actions.append("right")
        actions.append("right")
        actions.append("right")

        if energy > 50:
            actions.append("right")
            actions.append("right")
            actions.append("right")
            actions.append("right")
            actions.append("right")
            actions.append("right")
        if curiosity > 70:
            actions.append("think")

        actions.append("sleep 1")

        self.do_actions(actions, mood)

    def sing(self, mood):
        actions = []
        volume = mood["volume"]
        whimsy = mood["whimsy"]

        if volume > 60:
            actions.append("celebrate")
            actions.append("honk")
            actions.append("celebrate")
            actions.append("honk")
            actions.append("honk")
        else:
            actions.append("act cute")

        if whimsy > 70:
            actions.append("wave hands")

        actions.append("sleep 2")

        self.do_actions(actions, mood)

    def mutter(self, mood):
        actions = []
        sociability = mood["sociability"]
        curiosity = mood["curiosity"]

        if sociability < 30:
            actions.append("keep think")
        else:
            actions.append("think")

        if curiosity > 50:
            actions.append("rub hands")

        actions.append("sleep 1")

        self.do_actions(actions, mood)

    def dance(self, mood):
        actions = []
        energy = mood["energy"]
        whimsy = mood["whimsy"]

        actions.append("twist body")
        actions.append("celebrate")
        actions.append("twist body")
        actions.append("celebrate")

        if whimsy > 50:
            actions.append("forward 3 90")
            actions.append("backward 3 90")
            actions.append("forward 2 70")
            actions.append("backward 3 80")
            actions.append("forward 10 90")
            actions.append("backward 10 90")
        else:
            actions.append("right")
            actions.append("left")
            actions.append("backward 10 90")
            actions.append("right")
            actions.append("left")
            actions.append("backward 10 90")

        if mood["volume"] > 60:
            actions.append("celebrate")
            actions.append("honk")
            actions.append("celebrate")
            actions.append("honk")
            actions.append("honk")
            actions.append("celebrate")
            actions.append("honk")
            actions.append("celebrate")
            actions.append("celebrate")
            actions.append("honk")

        actions.append("sleep 2")

        self.do_actions(actions, mood)

    def run_idle_loop(self, cycles=1):
        """Run autonomous behaviors with vocalizations"""
        if not self.nevil.auto_enabled:
            return
        
        current_time = time.time()
        # Add cooldown period of 5 seconds after last interaction
        if current_time - self.last_interaction_time < 5:
            return
        
        for _ in range(cycles):
            # Check auto_enabled at start of each cycle
            if not self.nevil.auto_enabled:
                break
            
            rando = random.random()
            print(f"\n[auto] Random value: {rando:.3f} ({'GPT' if rando < 0.25 else 'Behavior'})")
            
            if rando < 0.25:  # 25% chance of GPT
                use_vision = random.random() < (self.current_mood.get('curiosity', 50) / 100)
                prompt = self.get_auto_prompt(use_vision)
                
                # Check auto_enabled before making GPT call
                if not self.nevil.auto_enabled:
                    break
                
                actions, message = self.nevil.call_GPT(prompt, use_image=use_vision)
                
                # Check auto_enabled before processing response
                if not self.nevil.auto_enabled:
                    break
                
                # Handle speech output
                if message:
                    self.nevil.handle_TTS_generation(message)
                    print(f"[auto speech] {message}")
                
                # Handle actions one at a time like in behaviors
                if actions:
                    self.do_actions(actions)
            else:
                # Use behavior system with vocalizations
                print(f"\n[auto] Mood: {self.current_mood_name}")
                weights = self.compute_behavior_weights(
                    self.BEHAVIOR_BASE_WEIGHTS,
                    self.current_mood,
                    self.BEHAVIOR_TRAIT_BIASES
                )
                behavior_name = self.weighted_choice(weights)
                print(f"[auto] Chosen behavior: {behavior_name}")
                print(f"[auto] Mood: {self.current_mood}")
                self.do_behavior(behavior_name, self.current_mood)
                
                # Wait for any speech to complete
                while True:
                    if not self.nevil.auto_enabled:
                        break
                    with self.nevil.speech_lock:
                        if not self.nevil.speech_loaded:
                            break
                    time.sleep(.01)

    def get_mood_param_str(self):
        traits = self.current_mood
        return f"mood={self.current_mood_name} volume={traits['volume']} curiosity={traits['curiosity']} sociability={traits['sociability']} whimsy={traits['whimsy']} energy={traits['energy']}"

    # -----------------------
    # Weight Calculation
    # -----------------------
    def compute_behavior_weights(self, base_weights, mood_profile, trait_biases):
        adjusted = {}
        for behavior, base_weight in base_weights.items():
            trait_influences = trait_biases.get(behavior, {})
            modifier = 1.0
            for trait, influence in trait_influences.items():
                trait_val = max(mood_profile.get(trait, 50) / 50, 0.01)
                modifier *= trait_val ** (influence - 1)
            adjusted[behavior] = base_weight * modifier
        return adjusted

    def weighted_choice(self, weight_dict):
        choices, weights = zip(*weight_dict.items())
        return random.choices(choices, weights=weights, k=1)[0]

    def pick_random_mood(self):
        name = random.choice(list(self.MOOD_PROFILES.keys()))
        return name, self.MOOD_PROFILES[name]

    def set_mood(self, mood_name):
        if mood_name in self.MOOD_PROFILES:
            self.current_mood_name = mood_name
            self.current_mood = self.MOOD_PROFILES[mood_name].copy()
            # Perform transition behavior
            self.mood_transition()
            return True
        return False

    def mood_transition(self):
        """Perform a transition behavior when mood changes"""
        actions = []
        if self.current_mood["energy"] > 70:
            actions.append("celebrate")
        elif self.current_mood["energy"] < 30:
            actions.append("depressed")
        else:
            actions.append("think")
        
        self.do_actions(actions, mood=f"transitioning to {self.current_mood_name}")

    def get_cycle_count(self):
        # Use energy and whimsy to determine number of cycles
        energy = self.current_mood.get("energy", 50)
        whimsy = self.current_mood.get("whimsy", 50)
        
        # More energy/whimsy = more likely to do more cycles
        base_cycles = random.randint(2, 10)
        mood_factor = (energy + whimsy) / 100  # 0 to 2 range
        cycles = max(2, min(10, int(base_cycles * mood_factor)))
        
        print(f"[system] Feeling {'energetic' if mood_factor > 1 else 'mellow'}, doing {cycles} cycles")
        return cycles

    BEHAVIOR_VOCALIZATIONS = {
        "explore": [
            "Hmm...", "What's that?", "Oh!", "Interesting...",
            "Let's see...", "Over here?", "This way!", "New thing!",
            "Ooooh!", "Check this out!", "Neat!", "Curious..."
        ],
        "rest": [
            "Ahhh...", "Mmm...", "Nice...", "Peaceful...",
            "Just chillin'", "Break time", "Resting...", "Quiet time..."
        ],
        "sleep": [
            "Zzzz...", "Sleepy...", "Zzz...", "Zzzz...",
            "Zzzz...", "Zzzz...", "Zzzz...", "Zzzz..."
        ],
        "fidget": [
            "Woah-woah-woah!", "Can't sit still!", "Jitters!",
            "Bouncy bouncy!", "Wiggles!", "Zoom zoom!", "Boing!"
        ],
        "address": [
            "Hello wall", "Hi there!", "Oh, hi!", "Fancy meeting you here!",
            "Well hello!", "Greetings!", "Hey hey!", "What's up!"
        ],
        "play": [
            "Wheee!", "Chicken look at this!", "Watch this!",
            "Fun fun fun!", "Yippee!", "Let's play!", "Playtime!",
            "Woo hoo!", "Yahoo!", "This is fun!", "Yay!"
        ],
        "panic": [
            "AAAHHHHH!", "git out da way!", "Coming through!",
            "Look out!", "Beep beep!", "Emergency!", "Yikes!",
            "Oh no oh no!", "Excuse me!", "Clear the way!"
        ],
        "circle": [
            "truckin'", "Round and round...", "Spin cycle!",
            "Dizzy...", "Wheeeee!", "Like a record baby!",
            "Round we go!", "Spinning!", "Circle time!"
        ],
        "sing": [
            "La la la...", "♪ ♫ ♬", "Doo bee doo...",
            "Beep boop song!", "Singing time!", "♪ Happy bot ♪",
            "Tra la la!", "♫ Robot tunes ♫", "Beep beat!"
        ],
        "mutter": [
            "Hmm...", "Mmmm...", "Let's see now...",
            "Maybe...", "Could be...", "Well well...",
            "So then...", "Huh...", "Right right..."
        ],
        "dance": [
            "Boogie!", "Let's dance!", "Got the moves!",
            "Robot dance!", "Groove time!", "Dancing!",
            "Watch these moves!", "Shake it!", "Robot rhythm!"
        ],
        "think": [
            "Processing...", "Computing...", "Analyzing...",
            "Deep thoughts...", "Robot thoughts...", "Calculating...",
            "Brain waves...", "Think think...", "Pondering..."
        ],
        "curious": [
            "What if...?", "I wonder...", "Could it be...?",
            "Let me see...", "How about...", "Maybe if...",
            "Curious...", "Investigating...", "Research time..."
        ]
    }

    AUTO_PROMPT_EXAMPLES = {
        "vision": [
            "Hey Dan",
            "Hey Danielle",
            "What up Chicken?!?",
            "wall",
            "couch",
            "Gainsborough!",
            "Delacroix!",
            "reading [book title]?",
            "sculpchah!",
            "nice painting!",
            "is that a cat?",
            "cool room!"
        ],
        "general": [
            "Hmm",
            "Woahhhh",
            "beep boop bop",
            "oh",
            "yurrrrrrrr",
            "dang",
            "wut?",
            "Whazzat?",
            "It's Kaiser Soze!!!",
            "meh",
            "huh",
            "interesting..."
        ]
    }

    def get_auto_prompt(self, use_vision):
        """Generate appropriate prompt for auto mode"""
        example_list = self.AUTO_PROMPT_EXAMPLES["vision" if use_vision else "general"]
        examples = random.sample(example_list, 3)
        prompt = (
            "Running in autonomous mode. Respond with a VERY brief comment "
            f"like '{examples[0]}' or '{examples[1]}' or '{examples[2]}'. "
            "Keep it to 1-5 words usually."
        )
        if use_vision:
            prompt += "Comment on things you see, or let them inspire your commentary or exclamations or questions or utterances."
        else:
            prompt += "What's on your mind?"
        return prompt

    def do_behavior(self, behavior_name, mood):
        """Execute behavior with possible vocalization"""
        behavior_func = self.BEHAVIOR_FUNCTIONS.get(behavior_name)
        if behavior_func:
            # Maybe add vocalization 
            vocalizations = self.BEHAVIOR_VOCALIZATIONS.get(behavior_name, [])
            vocalization_chance = (self.current_mood["energy"] + self.current_mood["whimsy"] + self.current_mood["sociability"] + self.current_mood["curiosity"] + self.current_mood["volume"]) / 500
            
            print(f"[vocal] Chance {vocalization_chance:.2f} from traits: energy={self.current_mood['energy']} whimsy={self.current_mood['whimsy']} sociability={self.current_mood['sociability']} curiosity={self.current_mood['curiosity']} volume={self.current_mood['volume']}")
            print(f"[vocal] Vocalizations: {vocalizations}")
            if vocalizations and random.random() < vocalization_chance:
                message = random.choice(vocalizations)
                self.nevil.handle_TTS_generation(message)
                print(f"[auto vocalization] {message}")
            
            # Do the behavior
            behavior_func(mood)

    def do_actions(self, actions, mood=None):
        """Queue multiple actions for execution"""
        if not actions:
            return
        
        # Log the actions
        action_str = ", ".join(actions)
        print(f"[actions] Queueing: {action_str}" + (f" ({mood})" if mood else ""))
        
        # Queue the actions
        with self.nevil.action_lock:
            self.nevil.actions_to_be_done = actions
            self.nevil.action_status = 'actions'
        
        # Wait for completion
        while True:
            if not self.nevil.auto_enabled:
                break
            with self.nevil.action_lock:
                if self.nevil.action_status != 'actions':
                    break
            time.sleep(.01)

