"""
Reward function and shaping constants for the Wordle sequencing task.

This module is imported by both task_sequencer.py (evaluation) and
training_env/wordle_env.py (simulation step). Separating it here means
the reward logic is visible in one place without loading any training code.

Reward design intent
--------------------
Correct task completion dominates travel-distance optimisation:
  - Completing the word alone is worth +100.
  - Each correct placement is +20.
  - Maximum travel cost per step (full workspace diagonal ~28 m RL space)
    is approximately −56, well below the +100 completion bonus.
  - The competition bonus/penalty (+10/−10 per metre vs greedy) encourages
    the policy to find shorter paths than the greedy baseline.
"""

# ============================================================
# Reward shaping constants
# ============================================================
WORD_COMPLETE_BONUS       = 100.0   # all required Wordle slots correctly filled
CORRECT_PLACEMENT_BONUS   =  20.0   # per newly correct Wordle slot (awarded once per slot)
CLEARING_BONUS            =  15.0   # clearing a wrong letter out of a Wordle slot to staging
WRONG_SLOT_PENALTY        = -20.0   # placing a letter in the wrong Wordle slot
MOVE_CORRECT_OUT_PENALTY  = -10.0   # evicting a correctly-placed letter from its slot
STEP_PENALTY              =  -1.0   # per step — penalises inefficient sequences
TRAVEL_COST_SCALE         =  -2.0   # multiplied by travel distance (metres) per step
COMPETITION_SCALE         =  10.0   # reward per metre RL beats greedy (negative if RL loses)


def custom_reward(
    placing_correct:       bool,
    placing_wrong_wordle:  bool,
    clearing_to_staging:   bool,
    moving_correct_out:    bool,
    word_complete:         bool,
    travel_distance:       float,
    slot_already_rewarded: bool,
    **kwargs,
) -> float:
    """
    Compute the scalar reward for a single pick-and-place action.

    Parameters
    ----------
    placing_correct       : letter matches the target at the destination Wordle slot
    placing_wrong_wordle  : letter does NOT match the target at the destination Wordle slot
    clearing_to_staging   : a letter is being moved out of a Wordle slot to staging
    moving_correct_out    : a correctly-placed letter is being evicted from its slot
    word_complete         : all required Wordle slots are now correctly filled
    travel_distance       : robot → source → destination, in RL-space metres
    slot_already_rewarded : the destination slot has already received a correct-placement bonus
    **kwargs              : extra context from the env (source_is_wordle, dest_is_wordle,
                            source_is_staging, dest_is_staging, step_count)

    Returns
    -------
    float : scalar reward for this step
    """
    reward = STEP_PENALTY
    reward += TRAVEL_COST_SCALE * travel_distance

    if moving_correct_out:
        reward += MOVE_CORRECT_OUT_PENALTY

    if clearing_to_staging and not moving_correct_out:
        reward += CLEARING_BONUS

    if placing_correct and not slot_already_rewarded:
        reward += CORRECT_PLACEMENT_BONUS

    if placing_wrong_wordle:
        reward += WRONG_SLOT_PENALTY

    if word_complete:
        reward += WORD_COMPLETE_BONUS

    return reward
