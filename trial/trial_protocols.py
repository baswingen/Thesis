"""
Trial Protocols Module
=======================
Predefined trial protocols and exercise definitions.

Contains standard protocols for:
- Basic hand/wrist movements
- MVC calibration
- Gesture recognition training
- Custom protocols
"""

from typing import List, Dict, Any, Optional


# =============================================================================
# PROTOCOL DEFINITIONS
# =============================================================================

BASIC_PROTOCOL = [
    {
        'name': 'Baseline_Rest',
        'duration': 10.0,
        'instruction': 'Relax your muscles completely.\nKeep your hand in a neutral position.',
        'rest_after': 2.0,
        'repetitions': 1,
        'description': 'Baseline resting state for normalization',
    },
    {
        'name': 'Wrist_Flexion',
        'duration': 5.0,
        'instruction': 'Slowly flex your wrist (bend hand downward).\nHold the position steadily.',
        'rest_after': 3.0,
        'repetitions': 3,
        'description': 'Wrist flexion movement',
    },
    {
        'name': 'Wrist_Extension',
        'duration': 5.0,
        'instruction': 'Slowly extend your wrist (bend hand upward).\nHold the position steadily.',
        'rest_after': 3.0,
        'repetitions': 3,
        'description': 'Wrist extension movement',
    },
    {
        'name': 'Hand_Grip',
        'duration': 5.0,
        'instruction': 'Close your hand into a fist.\nGrip with moderate force (50%).',
        'rest_after': 3.0,
        'repetitions': 3,
        'description': 'Hand grip/grasp',
    },
    {
        'name': 'Hand_Open',
        'duration': 5.0,
        'instruction': 'Fully extend all fingers.\nSpread fingers apart.',
        'rest_after': 3.0,
        'repetitions': 3,
        'description': 'Hand open/extension',
    },
]


PRONATION_SUPINATION_PROTOCOL = [
    {
        'name': 'Baseline_Rest',
        'duration': 10.0,
        'instruction': 'Relax your muscles completely.\nKeep your forearm in neutral position.',
        'rest_after': 2.0,
        'repetitions': 1,
        'description': 'Baseline resting state',
    },
    {
        'name': 'Pronation',
        'duration': 5.0,
        'instruction': 'Rotate your forearm (palm down).\nHold the position.',
        'rest_after': 3.0,
        'repetitions': 5,
        'description': 'Forearm pronation',
    },
    {
        'name': 'Supination',
        'duration': 5.0,
        'instruction': 'Rotate your forearm (palm up).\nHold the position.',
        'rest_after': 3.0,
        'repetitions': 5,
        'description': 'Forearm supination',
    },
]


MVC_CALIBRATION_PROTOCOL = [
    {
        'name': 'MVC_Wrist_Flexion',
        'duration': 5.0,
        'instruction': 'MAXIMUM wrist flexion!\nContract as hard as you can.',
        'rest_after': 10.0,
        'repetitions': 3,
        'description': 'Maximum voluntary contraction - wrist flexion',
        'is_mvc': True,
    },
    {
        'name': 'MVC_Wrist_Extension',
        'duration': 5.0,
        'instruction': 'MAXIMUM wrist extension!\nContract as hard as you can.',
        'rest_after': 10.0,
        'repetitions': 3,
        'description': 'Maximum voluntary contraction - wrist extension',
        'is_mvc': True,
    },
    {
        'name': 'MVC_Hand_Grip',
        'duration': 5.0,
        'instruction': 'MAXIMUM grip force!\nSqueeze as hard as you can.',
        'rest_after': 10.0,
        'repetitions': 3,
        'description': 'Maximum voluntary contraction - hand grip',
        'is_mvc': True,
    },
]


GESTURE_RECOGNITION_PROTOCOL = [
    {
        'name': 'Rest',
        'duration': 5.0,
        'instruction': 'Rest position.\nHand relaxed.',
        'rest_after': 2.0,
        'repetitions': 5,
        'description': 'Rest/neutral position',
    },
    {
        'name': 'Fist',
        'duration': 3.0,
        'instruction': 'Make a fist.',
        'rest_after': 2.0,
        'repetitions': 10,
        'description': 'Closed fist gesture',
    },
    {
        'name': 'Open_Hand',
        'duration': 3.0,
        'instruction': 'Open hand, fingers extended.',
        'rest_after': 2.0,
        'repetitions': 10,
        'description': 'Open hand gesture',
    },
    {
        'name': 'Pinch',
        'duration': 3.0,
        'instruction': 'Pinch thumb and index finger.',
        'rest_after': 2.0,
        'repetitions': 10,
        'description': 'Pinch grip',
    },
    {
        'name': 'Point',
        'duration': 3.0,
        'instruction': 'Point with index finger extended.',
        'rest_after': 2.0,
        'repetitions': 10,
        'description': 'Pointing gesture',
    },
    {
        'name': 'Thumb_Up',
        'duration': 3.0,
        'instruction': 'Thumbs up gesture.',
        'rest_after': 2.0,
        'repetitions': 10,
        'description': 'Thumbs up',
    },
]


FULL_PROTOCOL = [
    {
        'name': 'Baseline_Rest',
        'duration': 10.0,
        'instruction': 'Relax your muscles completely.\nKeep your hand in a neutral position.',
        'rest_after': 2.0,
        'repetitions': 1,
        'description': 'Baseline resting state',
    },
    {
        'name': 'Wrist_Flexion',
        'duration': 5.0,
        'instruction': 'Flex your wrist (bend hand downward).\nHold the position.',
        'rest_after': 3.0,
        'repetitions': 5,
        'description': 'Wrist flexion',
    },
    {
        'name': 'Wrist_Extension',
        'duration': 5.0,
        'instruction': 'Extend your wrist (bend hand upward).\nHold the position.',
        'rest_after': 3.0,
        'repetitions': 5,
        'description': 'Wrist extension',
    },
    {
        'name': 'Hand_Grip',
        'duration': 5.0,
        'instruction': 'Close your hand into a fist.\nGrip with moderate force.',
        'rest_after': 3.0,
        'repetitions': 5,
        'description': 'Hand grip',
    },
    {
        'name': 'Hand_Open',
        'duration': 5.0,
        'instruction': 'Fully extend all fingers.\nSpread fingers apart.',
        'rest_after': 3.0,
        'repetitions': 5,
        'description': 'Hand open',
    },
    {
        'name': 'Pronation',
        'duration': 5.0,
        'instruction': 'Rotate your forearm (palm down).\nHold the position.',
        'rest_after': 3.0,
        'repetitions': 5,
        'description': 'Forearm pronation',
    },
    {
        'name': 'Supination',
        'duration': 5.0,
        'instruction': 'Rotate your forearm (palm up).\nHold the position.',
        'rest_after': 3.0,
        'repetitions': 5,
        'description': 'Forearm supination',
    },
    {
        'name': 'Radial_Deviation',
        'duration': 5.0,
        'instruction': 'Tilt your wrist toward the thumb side.\nHold the position.',
        'rest_after': 3.0,
        'repetitions': 5,
        'description': 'Radial deviation',
    },
    {
        'name': 'Ulnar_Deviation',
        'duration': 5.0,
        'instruction': 'Tilt your wrist toward the pinky side.\nHold the position.',
        'rest_after': 3.0,
        'repetitions': 5,
        'description': 'Ulnar deviation',
    },
]


# =============================================================================
# PROTOCOL REGISTRY
# =============================================================================

PROTOCOLS = {
    'basic': BASIC_PROTOCOL,
    'pronation_supination': PRONATION_SUPINATION_PROTOCOL,
    'mvc_calibration': MVC_CALIBRATION_PROTOCOL,
    'gesture_recognition': GESTURE_RECOGNITION_PROTOCOL,
    'full': FULL_PROTOCOL,
}


# =============================================================================
# PROTOCOL UTILITIES
# =============================================================================

def get_protocol(name: str) -> List[Dict[str, Any]]:
    """
    Get a protocol by name.
    
    Args:
        name: Protocol name ('basic', 'mvc_calibration', etc.)
    
    Returns:
        List of exercise dictionaries
    
    Raises:
        ValueError: If protocol not found
    """
    if name not in PROTOCOLS:
        raise ValueError(f"Protocol '{name}' not found. Available: {list(PROTOCOLS.keys())}")
    
    return PROTOCOLS[name]


def list_protocols() -> List[str]:
    """
    List all available protocol names.
    
    Returns:
        List of protocol names
    """
    return list(PROTOCOLS.keys())


def validate_protocol(protocol: List[Dict[str, Any]]) -> bool:
    """
    Validate a protocol definition.
    
    Args:
        protocol: List of exercise dictionaries
    
    Returns:
        True if valid
    
    Raises:
        ValueError: If protocol is invalid
    """
    if not protocol or len(protocol) == 0:
        raise ValueError("Protocol must contain at least one exercise")
    
    required_fields = ['name', 'duration', 'instruction']
    
    for i, exercise in enumerate(protocol):
        for field in required_fields:
            if field not in exercise:
                raise ValueError(f"Exercise {i} missing required field '{field}'")
        
        if exercise['duration'] <= 0:
            raise ValueError(f"Exercise {i} duration must be positive")
        
        # Set defaults
        if 'rest_after' not in exercise:
            exercise['rest_after'] = 3.0
        if 'repetitions' not in exercise:
            exercise['repetitions'] = 1
        if 'description' not in exercise:
            exercise['description'] = exercise['name']
        if 'is_mvc' not in exercise:
            exercise['is_mvc'] = False
    
    return True


def expand_protocol(protocol: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Expand protocol by repeating exercises according to 'repetitions' field.
    
    Args:
        protocol: List of exercise dictionaries
    
    Returns:
        Expanded list with repetitions as separate trials
    """
    expanded = []
    
    for exercise in protocol:
        repetitions = exercise.get('repetitions', 1)
        
        for rep in range(repetitions):
            # Create copy with repetition info
            ex_copy = exercise.copy()
            ex_copy['repetition'] = rep + 1
            ex_copy['total_repetitions'] = repetitions
            
            # Adjust name to include repetition
            if repetitions > 1:
                ex_copy['display_name'] = f"{exercise['name']} (Rep {rep + 1}/{repetitions})"
            else:
                ex_copy['display_name'] = exercise['name']
            
            expanded.append(ex_copy)
    
    return expanded


def calculate_protocol_duration(protocol: List[Dict[str, Any]]) -> float:
    """
    Calculate total duration of protocol including rest periods.
    
    Args:
        protocol: List of exercise dictionaries
    
    Returns:
        Total duration in seconds
    """
    total = 0.0
    
    for exercise in protocol:
        duration = exercise['duration']
        rest_after = exercise.get('rest_after', 0.0)
        repetitions = exercise.get('repetitions', 1)
        
        total += (duration + rest_after) * repetitions
    
    return total


def print_protocol_summary(protocol: List[Dict[str, Any]], name: str = "Protocol"):
    """
    Print a summary of the protocol.
    
    Args:
        protocol: List of exercise dictionaries
        name: Protocol name
    """
    print(f"\n{'='*70}")
    print(f"{name.upper()} PROTOCOL SUMMARY")
    print(f"{'='*70}")
    
    expanded = expand_protocol(protocol)
    total_duration = calculate_protocol_duration(protocol)
    
    print(f"\nTotal exercises: {len(protocol)}")
    print(f"Total trials (with repetitions): {len(expanded)}")
    print(f"Estimated duration: {total_duration:.1f} seconds ({total_duration/60:.1f} minutes)")
    
    print(f"\nExercise List:")
    for i, exercise in enumerate(protocol, 1):
        name = exercise['name']
        duration = exercise['duration']
        rest = exercise.get('rest_after', 0)
        reps = exercise.get('repetitions', 1)
        
        print(f"  {i}. {name}")
        print(f"     Duration: {duration}s × {reps} reps, Rest: {rest}s")
        print(f"     Instruction: {exercise['instruction'].split('.')[0]}")
    
    print(f"{'='*70}\n")


def create_custom_protocol(exercises: List[str], 
                          durations: List[float],
                          instructions: Optional[List[str]] = None,
                          rest_after: float = 3.0,
                          repetitions: int = 1) -> List[Dict[str, Any]]:
    """
    Create a custom protocol from simple parameters.
    
    Args:
        exercises: List of exercise names
        durations: List of durations (seconds) for each exercise
        instructions: Optional list of instructions (auto-generated if None)
        rest_after: Rest period after each exercise
        repetitions: Number of repetitions for each exercise
    
    Returns:
        Protocol list
    
    Example:
        >>> protocol = create_custom_protocol(
        ...     exercises=['Rest', 'Flexion', 'Extension'],
        ...     durations=[10, 5, 5],
        ...     repetitions=3
        ... )
    """
    if len(exercises) != len(durations):
        raise ValueError("exercises and durations must have same length")
    
    if instructions is None:
        instructions = [f"Perform {name}" for name in exercises]
    elif len(instructions) != len(exercises):
        raise ValueError("instructions must match exercises length")
    
    protocol = []
    for name, duration, instruction in zip(exercises, durations, instructions):
        protocol.append({
            'name': name,
            'duration': duration,
            'instruction': instruction,
            'rest_after': rest_after,
            'repetitions': repetitions,
            'description': name,
        })
    
    return protocol


# =============================================================================
# DEMO
# =============================================================================

if __name__ == '__main__':
    print("Trial Protocols Module")
    print("="*70)
    
    print("\nAvailable Protocols:")
    for name in list_protocols():
        protocol = get_protocol(name)
        num_exercises = len(protocol)
        duration = calculate_protocol_duration(protocol)
        print(f"  - {name}: {num_exercises} exercises, ~{duration/60:.1f} min")
    
    # Print basic protocol details
    print_protocol_summary(BASIC_PROTOCOL, "Basic")
    
    # Example custom protocol
    print("\nCreating custom protocol...")
    custom = create_custom_protocol(
        exercises=['Rest', 'Movement_A', 'Movement_B'],
        durations=[5, 3, 3],
        instructions=['Relax', 'Perform movement A', 'Perform movement B'],
        repetitions=2
    )
    print_protocol_summary(custom, "Custom")
