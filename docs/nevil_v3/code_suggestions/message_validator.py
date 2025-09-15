# nevil_framework/message_validator.py

import jsonschema
from typing import Dict, Any, Optional
from dataclasses import dataclass
import time

@dataclass
class ValidationResult:
    valid: bool
    errors: list = None
    warnings: list = None
    validation_time: float = 0.0

class MessageValidator:
    """
    Runtime message validation system.

    Validates messages against schemas defined in .messages files
    with performance monitoring and error reporting.
    """

    def __init__(self):
        self.schemas = {}  # topic -> schema
        self.validation_stats = {}  # topic -> stats
        self.strict_mode = False  # Set to True for development

    def load_schema(self, topic: str, schema: Dict[str, Any]):
        """Load message schema for a topic"""
        try:
            # Convert our simple schema format to JSON Schema
            json_schema = self._convert_to_json_schema(schema)

            # Validate the schema itself
            jsonschema.Draft7Validator.check_schema(json_schema)

            self.schemas[topic] = json_schema
            self.validation_stats[topic] = {
                "validated_count": 0,
                "error_count": 0,
                "warning_count": 0,
                "total_validation_time": 0.0
            }

        except Exception as e:
            print(f"Error loading schema for topic {topic}: {e}")

    def validate_message(self, topic: str, message_data: Any) -> ValidationResult:
        """Validate message against topic schema"""
        start_time = time.time()

        if topic not in self.schemas:
            return ValidationResult(
                valid=True,  # No schema = no validation required
                warnings=[f"No schema defined for topic {topic}"]
            )

        try:
            schema = self.schemas[topic]
            validator = jsonschema.Draft7Validator(schema)
            errors = list(validator.iter_errors(message_data))

            validation_time = time.time() - start_time

            # Update statistics
            stats = self.validation_stats[topic]
            stats["validated_count"] += 1
            stats["total_validation_time"] += validation_time

            if errors:
                stats["error_count"] += 1
                return ValidationResult(
                    valid=False,
                    errors=[f"{error.json_path}: {error.message}" for error in errors],
                    validation_time=validation_time
                )
            else:
                return ValidationResult(
                    valid=True,
                    validation_time=validation_time
                )

        except Exception as e:
            return ValidationResult(
                valid=False,
                errors=[f"Validation exception: {e}"],
                validation_time=time.time() - start_time
            )

    def _convert_to_json_schema(self, schema: Dict[str, Any]) -> Dict[str, Any]:
        """Convert our simple schema format to JSON Schema"""
        json_schema = {
            "type": "object",
            "properties": {},
            "required": []
        }

        for field_name, field_def in schema.items():
            if isinstance(field_def, dict):
                prop = {"type": field_def.get("type", "string")}

                if field_def.get("required", False):
                    json_schema["required"].append(field_name)

                if "min" in field_def:
                    prop["minimum"] = field_def["min"]
                if "max" in field_def:
                    prop["maximum"] = field_def["max"]
                if "max_length" in field_def:
                    prop["maxLength"] = field_def["max_length"]
                if "allowed" in field_def:
                    prop["enum"] = field_def["allowed"]

                json_schema["properties"][field_name] = prop

        return json_schema

    def get_validation_stats(self) -> Dict[str, Any]:
        """Get validation statistics"""
        return self.validation_stats.copy()

# Enhanced Message Bus with Validation
class MessageBus:
    def __init__(self, max_queue_size: int = 1000, enable_validation: bool = True):
        # ... existing initialization ...
        self.message_validator = MessageValidator() if enable_validation else None
        self.validation_enabled = enable_validation

    def publish(self, message: Message) -> bool:
        """Enhanced publish with optional validation"""
        try:
            # Validate message if validation is enabled
            if self.validation_enabled and self.message_validator:
                validation_result = self.message_validator.validate_message(
                    message.topic, message.data
                )

                if not validation_result.valid:
                    error_msg = f"Message validation failed for topic {message.topic}: {validation_result.errors}"
                    print(f"WARNING: {error_msg}")

                    # In strict mode, reject invalid messages
                    if self.message_validator.strict_mode:
                        self.stats['messages_dropped'] += 1
                        return False

            # Continue with existing publish logic
            return self._publish_validated_message(message)

        except Exception as e:
            print(f"Error in enhanced publish: {e}")
            return False