# ACTUAL Servo Calibration Fix - Technical Root Cause

## The Real Problem
The 45-degree servo issue occurs because **v2.0's fallback configuration system never reads the actual config file**.

### Technical Analysis

**v1.0 (Working):**
```python
# Line 50 - ALWAYS uses fileDB
self.config_flie = fileDB(config, 777, os.getlogin())

# Line 57 - Reads actual config file
self.dir_cali_val = float(self.config_flie.get("picarx_dir_servo", default_value=0))
```

**v2.0 (Broken):**
```python
# Lines 67-78 - Creates fallback when fileDB is None
if fileDB is not None:
    self.config_flie = fileDB(config, 777, os.getlogin())
else:
    class FallbackConfig:
        def __init__(self):
            self._data = {}  # EMPTY DICT - NEVER READS FILE
        def get(self, key, default_value=None):
            return self._data.get(key, default_value)  # ALWAYS RETURNS DEFAULT
```

**The Bug:** FallbackConfig never reads `/opt/picar-x/picar-x.conf`, so calibration values are ALWAYS 0.

## The Fix

Replace the broken FallbackConfig with one that actually reads the config file:

```python
class FallbackConfig:
    def __init__(self, config_path):
        self.config_path = config_path
        self._data = {}
        self._load_config()
    
    def _load_config(self):
        """Actually read the config file"""
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if line and not line.startswith('#') and '=' in line:
                            key, value = line.split('=', 1)
                            self._data[key.strip()] = value.strip()
            except Exception as e:
                print(f"Warning: Could not read config file: {e}")
    
    def get(self, key, default_value=None):
        return self._data.get(key, default_value)
    
    def set(self, key, value):
        self._data[key] = str(value)
        # Write back to file
        self._save_config()
    
    def _save_config(self):
        """Save config back to file"""
        try:
            with open(self.config_path, 'w') as f:
                f.write("# PiCar-X Configuration File\n")
                for key, value in self._data.items():
                    f.write(f"{key}={value}\n")
        except Exception as e:
            print(f"Warning: Could not save config file: {e}")
```

## Immediate Fix

Replace lines 70-78 in `src/nevil_navigation/nevil_navigation/picarx.py`:


## Other Possible Root Causes of servo issue

I suspect that the problem may be:
1. Lack of sudo run on navigation (v1.0 calibration requires sudo - see v1.0)
2. calibration or initialization files in /nevil2env, /projects, or other non root, non-src startup files. 