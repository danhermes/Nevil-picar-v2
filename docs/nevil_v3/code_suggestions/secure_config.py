# nevil_framework/secure_config.py

import os
import re
import base64
import hashlib
from cryptography.fernet import Fernet
from typing import Dict, Any, List, Optional

class SecureConfigManager:
    """
    Enhanced configuration management with security features.

    Provides encryption for sensitive values, credential masking,
    and secure environment variable handling.
    """

    def __init__(self, master_key: Optional[str] = None):
        self.master_key = master_key or os.getenv('NEVIL_MASTER_KEY')
        self.cipher = None

        if self.master_key:
            key = base64.urlsafe_b64encode(
                hashlib.sha256(self.master_key.encode()).digest()[:32]
            )
            self.cipher = Fernet(key)

        # Sensitive key patterns
        self.sensitive_patterns = [
            r'.*key.*',
            r'.*secret.*',
            r'.*password.*',
            r'.*token.*',
            r'.*credential.*',
            r'.*api.*key.*'
        ]

    def mask_sensitive_values(self, config: Dict[str, Any],
                            mask_char: str = '*') -> Dict[str, Any]:
        """Recursively mask sensitive values in configuration"""
        masked_config = {}

        for key, value in config.items():
            if isinstance(value, dict):
                masked_config[key] = self.mask_sensitive_values(value, mask_char)
            elif self._is_sensitive_key(key):
                if isinstance(value, str) and value:
                    # Show first 2 and last 2 characters
                    if len(value) > 6:
                        masked_config[key] = value[:2] + mask_char * (len(value) - 4) + value[-2:]
                    else:
                        masked_config[key] = mask_char * len(value)
                else:
                    masked_config[key] = mask_char * 8
            else:
                masked_config[key] = value

        return masked_config

    def encrypt_sensitive_value(self, value: str) -> str:
        """Encrypt a sensitive value"""
        if not self.cipher:
            raise ValueError("No master key configured for encryption")

        encrypted = self.cipher.encrypt(value.encode())
        return f"ENC:{base64.urlsafe_b64encode(encrypted).decode()}"

    def decrypt_sensitive_value(self, encrypted_value: str) -> str:
        """Decrypt a sensitive value"""
        if not self.cipher:
            raise ValueError("No master key configured for decryption")

        if not encrypted_value.startswith("ENC:"):
            return encrypted_value  # Not encrypted

        encrypted_data = base64.urlsafe_b64decode(encrypted_value[4:])
        return self.cipher.decrypt(encrypted_data).decode()

    def process_environment_variables(self, config: Dict[str, Any]) -> Dict[str, Any]:
        """Process environment variables with security checks"""
        processed_config = {}

        for key, value in config.items():
            if isinstance(value, dict):
                processed_config[key] = self.process_environment_variables(value)
            elif isinstance(value, str):
                # Handle encrypted values
                if value.startswith("ENC:"):
                    try:
                        processed_config[key] = self.decrypt_sensitive_value(value)
                    except Exception as e:
                        print(f"Warning: Failed to decrypt {key}: {e}")
                        processed_config[key] = value

                # Handle environment variable substitution
                elif value.startswith("${") and value.endswith("}"):
                    env_var = value[2:-1]
                    default_value = ""

                    # Handle ${VAR:-default} syntax
                    if ":-" in env_var:
                        env_var, default_value = env_var.split(":-", 1)

                    env_value = os.getenv(env_var, default_value)

                    # Validate sensitive environment variables
                    if self._is_sensitive_key(env_var) and not env_value:
                        print(f"WARNING: Sensitive environment variable {env_var} is not set")

                    processed_config[key] = env_value
                else:
                    processed_config[key] = value
            else:
                processed_config[key] = value

        return processed_config

    def validate_environment_variables(self, required_vars: List[str]) -> List[str]:
        """Validate that required environment variables are set"""
        missing_vars = []

        for var in required_vars:
            value = os.getenv(var)
            if not value:
                missing_vars.append(var)
            elif self._is_sensitive_key(var):
                # Additional validation for sensitive vars
                if len(value) < 10:  # Reasonable minimum for API keys
                    print(f"WARNING: {var} appears too short for a secure credential")

        return missing_vars

    def _is_sensitive_key(self, key: str) -> bool:
        """Check if a key name indicates sensitive data"""
        key_lower = key.lower()
        return any(re.match(pattern, key_lower, re.IGNORECASE)
                  for pattern in self.sensitive_patterns)

    def get_sanitized_config_for_logging(self, config: Dict[str, Any]) -> str:
        """Get configuration suitable for logging (with sensitive data masked)"""
        import json
        masked_config = self.mask_sensitive_values(config)
        return json.dumps(masked_config, indent=2, sort_keys=True)