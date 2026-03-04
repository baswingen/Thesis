"""
Trial Logic
===========

Handles the state machine of the experimental trial.

Phases:
0. CONFIGURING: Waiting to start.
1. PHASE_1_PLACEMENT: Moving initial weights from off-matrix starting slots to random spots on the matrix.
2. PHASE_2_MOVEMENTS: Moving weights between random spots on the matrix until the trial is complete.
3. FINISHED: Trial is complete.
"""

import random
from typing import Dict, List, Optional
from enum import Enum, auto

from trial.trial_config import TRIAL_LOGIC_CONFIG, STARTING_SLOTS_CONFIG


class TrialState(Enum):
    CONFIGURING = auto()
    PHASE_1_PLACEMENT = auto()
    PHASE_2_MOVEMENTS = auto()
    ERROR_WRONG_PICKUP = auto()
    ERROR_WRONG_PLACEMENT = auto()
    ERROR_MISSING_WEIGHT = auto()
    FINISHED = auto()


class TrialLogic:
    """State machine governing the placement and movement of weights on the matrix."""
    
    # 3x4 Matrix dimensions from dashboard setup.
    ROWS = 3
    COLS = 4
    TOTAL_SPOTS = ROWS * COLS

    def __init__(self):
        self.state = TrialState.CONFIGURING
        
        # Configuration
        self.max_movements = int(TRIAL_LOGIC_CONFIG.get("Movements", 15))
        
        # Initialize slots: (row, col) -> weight_in_kg or None
        self.matrix_state: Dict[tuple[int, int], Optional[float]] = {
            (r, c): None for r in range(self.ROWS) for c in range(self.COLS)
        }
        
        # Weights pending placement onto the matrix from visual slots
        # We assign an index to each pending weight corresponding to its slot (1-8)
        self.pending_weights: List[tuple[int, float]] = []
        for i, (slot_name, w) in enumerate(STARTING_SLOTS_CONFIG.items()):
            if w is not None:
                self.pending_weights.append((i, w))
        random.shuffle(self.pending_weights) # randomize placement order
        
        # Keep track of which initial slots are empty
        self.starting_slots_state = {i: w for i, w in enumerate(STARTING_SLOTS_CONFIG.values())}
        
        # Track movement counts per weight value to enable even distribution
        self.weight_move_counts: Dict[float, int] = {}
        for w in STARTING_SLOTS_CONFIG.values():
            if w is not None:
                self.weight_move_counts[w] = 0
        
        # Tracking progress
        self.movements_completed = 0
        
        # Current instruction variables
        self.current_instruction_text = "Press 'START TRIAL' to begin."
        self.target_spot: Optional[tuple[int, int]] = None  # The place to move to
        self.target_weight: Optional[float] = None          # The weight being moved
        self.target_slot_idx: Optional[int] = None          # The starting slot it came from (Phase 1)
        self.source_spot: Optional[tuple[int, int]] = None    # Where it's coming from (None if from off-matrix)
        
        # Error recovery state
        self.previous_state: Optional[TrialState] = None
        self.expected_recovery_spot: Optional[tuple[int, int]] = None
        self.error_weight_tracking: Optional[float] = None
        
        # Event logging queue: list of (t_pc, state_name, weight, src_r, src_c, tgt_r, tgt_c)
        self.events_queue: List[tuple] = []
        
        # Mask tracking to detect edge changes
        self.last_keys_mask = 0
        self.awaiting_pickup = False # For Phase 2
        self.start_recording_flag = False # Set True when transitioning to Phase 2

    def start(self):
        """Transition from configuring to Phase 1 Placement."""
        if self.state == TrialState.CONFIGURING:
            self.state = TrialState.PHASE_1_PLACEMENT
            self._next_placement()
        
    def get_instruction(self) -> str:
        """Returns the current instruction for the participant."""
        return self.current_instruction_text
        
    def get_target_highlight(self) -> Optional[tuple[int, int]]:
        """Returns the grid coords to highlight, or None."""
        return self.target_spot

    def _log_event(self, t_pc: float):
        """Appends the current state to the events queue."""
        state_name = self.state.name
        
        # For PHASE_2_MOVEMENTS, we split into AWAITING_PICKUP and AWAITING_PLACEMENT
        if self.state == TrialState.PHASE_2_MOVEMENTS:
            if self.awaiting_pickup:
                state_name = "PHASE_2_AWAITING_PICKUP"
            else:
                state_name = "PHASE_2_AWAITING_PLACEMENT"
                
        src_r, src_c = self.source_spot if self.source_spot else (-1, -1)
        tgt_r, tgt_c = self.target_spot if self.target_spot else (-1, -1)
        weight = self.target_weight if self.target_weight is not None else -1.0
        
        self.events_queue.append((
            t_pc, state_name, weight, src_r, src_c, tgt_r, tgt_c
        ))

    def update(self, keys_mask: int, t_pc: float):
        """
        Called repeatedly (e.g. from the dashboard fast_tick) with the latest keys_mask.
        We detect rising/falling edges on the matrix to update our state.
        
        t_pc is the precise timestamp (either PC time or STM32 synchronized time) 
        of when this keys_mask was sampled.
        
        Bit index = r * COLS + c.
        0 means empty, 1 means a weight is currently pressing the button.
        """
        if self.state in (TrialState.CONFIGURING, TrialState.FINISHED):
            return

        # Find changes
        changed = self.last_keys_mask ^ keys_mask
        
        if changed == 0:
            return # No change
            
        # Detect which specific buttons changed
        for r in range(self.ROWS):
            for c in range(self.COLS):
                bit_idx = r * self.COLS + c
                if (changed >> bit_idx) & 1:
                    pressed = bool((keys_mask >> bit_idx) & 1)
                    self._handle_button_change(r, c, pressed, t_pc)
                    
        self.last_keys_mask = keys_mask
        
        # 1. Continuous Validation: if a spot is supposed to have a weight but the button is NOT pressed, error!
        # Only validate during active phases (not during an error state)
        if self.state in (TrialState.PHASE_1_PLACEMENT, TrialState.PHASE_2_MOVEMENTS):
            for r in range(self.ROWS):
                for c in range(self.COLS):
                    spot = (r, c)
                    # Exclude the current source_spot if we are actively awaiting its pickup
                    if self.state == TrialState.PHASE_2_MOVEMENTS and self.awaiting_pickup and spot == self.source_spot:
                        continue
                        
                    w = self.matrix_state.get(spot)
                    idx = r * self.COLS + c
                    is_pressed = bool((keys_mask >> idx) & 1)

                    if w is not None:
                        # Should be pressed
                        if not is_pressed:
                            # It's lifted!
                            self.matrix_state[spot] = None
                            self._trigger_error(
                                TrialState.ERROR_MISSING_WEIGHT,
                                f"ERROR: Missing weight ({w}kg) from {chr(ord('A')+r)}{c+1}! Please replace it.",
                                spot, t_pc
                            )
                            self.error_weight_tracking = w
                            return # Stop processing edges until fixed
                    else:
                        # Should NOT be pressed
                        if is_pressed:
                            # Unrecognized extra weight
                            self._trigger_error(
                                TrialState.ERROR_WRONG_PLACEMENT,
                                f"ERROR: Unrecognized weight placed on {chr(ord('A')+r)}{c+1}! Please remove it.",
                                spot, t_pc
                            )
                            return # Stop processing edges until fixed
                            
        # 2. Process edge events (like normal placement/pickup)

    def _handle_button_change(self, r: int, c: int, pressed: bool, t_pc: float):
        """Process a specific button press or release."""
        spot = (r, c)
        
        if self.state == TrialState.PHASE_1_PLACEMENT:
            # We are waiting for target_weight to be placed at target_spot.
            if pressed:
                if spot == self.target_spot:
                    # Successfully placed!
                    self.matrix_state[spot] = self.target_weight
                    self._next_placement(t_pc)
                else:
                    # Wrong spot placed!
                    self._trigger_error(
                        TrialState.ERROR_WRONG_PLACEMENT,
                        f"ERROR: Placed {self.target_weight}kg in wrong spot! Please remove it.",
                        spot, t_pc
                    )
            elif not pressed and spot == self.target_spot:
                 # It was removed? Ignore for now, or flag error.
                 pass
                 
        elif self.state == TrialState.PHASE_2_MOVEMENTS:
            if self.awaiting_pickup:
                # We are waiting for the target_weight to be picked up from source_spot
                if not pressed:
                    if spot == self.source_spot:
                        self.matrix_state[spot] = None  # It's lifted
                        self.awaiting_pickup = False
                        
                        # Update instruction to tell them where to put it
                        tr, tc = self.target_spot
                        col_letter = chr(ord('A') + tr)
                        dest_str = f"{col_letter}{tc + 1}"
                        self.current_instruction_text = f"Place the {self.target_weight}kg weight on {dest_str}"
                        self._log_event(t_pc)
                    elif spot in self.matrix_state and self.matrix_state[spot] is not None:
                        # Picked up the wrong weight!
                        err_weight = self.matrix_state[spot]
                        self.matrix_state[spot] = None
                        self._trigger_error(
                            TrialState.ERROR_WRONG_PICKUP,
                            f"ERROR: Picked up wrong weight ({err_weight}kg)! Please put it back on {chr(ord('A')+spot[0])}{spot[1]+1}.",
                            spot, t_pc
                        )
                        self.error_weight_tracking = err_weight
            else:
                # We are waiting for it to be placed at target_spot
                if pressed:
                    if spot == self.target_spot:
                        # Successfully moved
                        self.matrix_state[spot] = self.target_weight
                        self.movements_completed += 1
                        
                        if self.movements_completed >= self.max_movements:
                            self._finish_trial(t_pc)
                        else:
                            self._next_movement(t_pc)
                    else:
                        # Placed in the wrong spot
                        self._trigger_error(
                            TrialState.ERROR_WRONG_PLACEMENT,
                            f"ERROR: Placed {self.target_weight}kg in wrong spot! Please remove it.",
                            spot, t_pc
                        )
                        
        elif self.state == TrialState.ERROR_WRONG_PICKUP:
            # Waiting for them to put the wrongly picked up weight BACK on the expected spot
            if pressed and spot == self.expected_recovery_spot:
                self.matrix_state[spot] = self.error_weight_tracking
                self._recover_from_error(t_pc)
                
        elif self.state == TrialState.ERROR_WRONG_PLACEMENT:
            # Waiting for them to lift the wrongly placed weight from the incorrect spot
            if not pressed and spot == self.expected_recovery_spot:
                self._recover_from_error(t_pc)
                
        elif self.state == TrialState.ERROR_MISSING_WEIGHT:
            # Waiting for them to replace the missing weight to its spot
            if pressed and spot == self.expected_recovery_spot:
                self.matrix_state[spot] = self.error_weight_tracking
                self._recover_from_error(t_pc)

    def _trigger_error(self, error_state: TrialState, msg: str, spot: tuple[int, int], t_pc: float):
        self.previous_state = self.state
        self.state = error_state
        self.expected_recovery_spot = spot
        self.error_message = msg
        self.current_instruction_text = msg
        self._log_event(t_pc)
        
    def _recover_from_error(self, t_pc: float):
        self.state = self.previous_state
        self.expected_recovery_spot = None
        self.error_weight_tracking = None # Clear tracking after recovery
        # Restore previous instruction implicitly because UI uses get_instruction()
        if self.state == TrialState.PHASE_2_MOVEMENTS:
            if self.awaiting_pickup:
                sr, sc = self.source_spot
                tr, tc = self.target_spot
                self.current_instruction_text = f"Move {self.target_weight}kg from {chr(ord('A')+sr)}{sc+1} to {chr(ord('A')+tr)}{tc+1}"
            else:
                tr, tc = self.target_spot
                self.current_instruction_text = f"Place the {self.target_weight}kg weight on {chr(ord('A')+tr)}{tc+1}"
        elif self.state == TrialState.PHASE_1_PLACEMENT:
            tr, tc = self.target_spot
            self.current_instruction_text = f"Place the {self.target_weight}kg weight on {chr(ord('A')+tr)}{tc+1}"
        self._log_event(t_pc)

    def _next_placement(self, t_pc: float = 0.0):
        """Pick the next weight from off-matrix and assign it a random empty spot."""
        # Also, if we just placed one, clear its original slot
        if self.target_slot_idx is not None:
             self.starting_slots_state[self.target_slot_idx] = None
             self.target_slot_idx = None
             
        if not self.pending_weights:
            # Phase 1 done! Move to Phase 2.
            self.state = TrialState.PHASE_2_MOVEMENTS
            self.start_recording_flag = True # Signal to manager
            self._next_movement(t_pc)
            return
            
        self.target_slot_idx, self.target_weight = self.pending_weights.pop(0)
        
        empty_spots = [pos for pos, w in self.matrix_state.items() if w is None]
        if not empty_spots:
            # Should not happen unless matrix is completely full
            self._finish_trial(t_pc)
            return
            
        self.target_spot = random.choice(empty_spots)
        tr, tc = self.target_spot
        
        # e.g A1, B2, etc. Row 0=A, Col 0=1
        col_letter = chr(ord('A') + tr)
        dest_str = f"{col_letter}{tc + 1}"
        
        self.current_instruction_text = f"Place the {self.target_weight}kg weight on {dest_str}"
        self.source_spot = None
        self.awaiting_pickup = False
        self._log_event(t_pc)

    def _next_movement(self, t_pc: float = 0.0):
        """Pick a random weight currently on the matrix and move it to a random empty spot."""
        occupied_spots = {pos: w for pos, w in self.matrix_state.items() if w is not None}
        empty_spots = [pos for pos, w in self.matrix_state.items() if w is None]
        
        if not occupied_spots or not empty_spots:
             self._finish_trial(t_pc)
             return

        # Find the minimum move count among currently available weights
        min_moves = min(self.weight_move_counts.get(w, 0) for w in occupied_spots.values())
        candidates = [(pos, w) for pos, w in occupied_spots.items() if self.weight_move_counts.get(w, 0) == min_moves]

        # Pick random weight to move
        self.source_spot, self.target_weight = random.choice(candidates)
        self.weight_move_counts[self.target_weight] = self.weight_move_counts.get(self.target_weight, 0) + 1
        
        # Pick random destination
        self.target_spot = random.choice(empty_spots)
        
        sr, sc = self.source_spot
        tr, tc = self.target_spot
        
        src_str = f"{chr(ord('A') + sr)}{sc + 1}"
        dest_str = f"{chr(ord('A') + tr)}{tc + 1}"
        
        self.current_instruction_text = f"Move {self.target_weight}kg from {src_str} to {dest_str}"
        self.awaiting_pickup = True
        self._log_event(t_pc)
        
    def _finish_trial(self, t_pc: float = 0.0):
        self.state = TrialState.FINISHED
        self.current_instruction_text = "Trial Complete! Stop Recording."
        self.target_spot = None
        self._log_event(t_pc)

    def get_progress_text(self) -> str:
        """Returns a string like 'Move 3/15' or 'Setup'."""
        if self.state in (TrialState.CONFIGURING, TrialState.PHASE_1_PLACEMENT):
            return "Setup"
        if self.state == TrialState.FINISHED:
            return "Done"
        return f"Move {self.movements_completed + 1} / {self.max_movements}"
