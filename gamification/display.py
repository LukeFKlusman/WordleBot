import sys
import time
import random

from constants import GOOD, BAD_POSITION, INCORRECT, EASTER_EGG_WORDS, EASTER_EGG_INSULTS, C

def colour_feedback(guess, feedback):
    """Returns a string showing guess letters with feedback symbols."""
    symbol_map = {GOOD: "[G]", BAD_POSITION: "[B]", INCORRECT: "[ ]"}
    return "  " + " ".join(symbol_map[fb] + guess[i].upper() for i, fb in enumerate(feedback))

def print_remaining_info(candidates):
    """Shows how many candidates remain after filtering."""
    count = len(candidates)
    if count == 0:
        print("  (No candidates left)")
    elif count <= 10:
        words = ", ".join(w.upper() for w in candidates)
        print(f"  ({count} word(s) left: {words})")
    else:
        print(f"  ({count} possible words still remaining)")

def print_colour_legend():
    print("  [G] = Good (correct position)")
    print("  [B] = Bad position (wrong spot, right letter)")
    print("  [ ] = Incorrect (not in word)\n")

def print_title():
    print("\n  ┌─────────────────────┐")
    print("  │    Wordle Solver     │")
    print("  └─────────────────────┘")

def slow_print(text, delay=0.03):
    """Prints text character by character for dramatic effect."""
    for ch in text:
        sys.stdout.write(ch)
        sys.stdout.flush()
        time.sleep(delay)
    print()

def trigger_easter_egg(word):
    """Absolutely roasts the player for picking an embarrassingly easy word."""
    print("\n  " + "=" * 54)
    time.sleep(0.4)
    slow_print("\n  *** PATHETIC WORD DETECTED ***", delay=0.05)
    time.sleep(0.5)

    insult = random.choice(EASTER_EGG_INSULTS).format(word=word.upper())
    slow_print(f"\n  {insult}", delay=0.03)
    time.sleep(0.3)

    slow_print("\n  I am EMBARRASSED to have even been given this task.", delay=0.03)
    slow_print(f"  A CHILD could guess {word.upper()}. A SLEEPING child.", delay=0.03)
    slow_print("  Please. For the love of all things holy. Try harder.", delay=0.03)

    time.sleep(0.4)
    print("\n  " + "=" * 54 + "\n")
    time.sleep(0.6)

def print_bug_report():
    """Shows the hidden easter egg words — dev reference."""
    print("\n  ── Easter Egg Words ────────────────")
    for word in sorted(EASTER_EGG_WORDS):
        print(f"  > {word.upper()}")
    print("  ────────────────────────────────────\n")


session_stats = {"games": 0, "total_attempts": 0, "best": None}

def update_stats(attempts):
    session_stats["games"] += 1
    session_stats["total_attempts"] += attempts
    if session_stats["best"] is None or attempts < session_stats["best"]:
        session_stats["best"] = attempts

def print_stats():
    g = session_stats["games"]
    if g == 0:
        print("\n  No games played yet this session.")
        return
    avg  = session_stats["total_attempts"] / g
    best = session_stats["best"]
    print("\n  ── Session Stats ───────────────────")
    print(f"  Games played : {g}")
    print(f"  Avg attempts : {avg:.1f}")
    print(f"  Best game    : {best} attempt(s)")
    print("  ────────────────────────────────────\n")
