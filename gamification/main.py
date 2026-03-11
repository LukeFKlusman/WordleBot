from constants    import GOOD, EASTER_EGG_WORDS
from dictionary   import load_dictionary
from wordle_logic import (
    score_guess_against_target,
    filter_candidates,
    choose_opening_guess,
    choose_best_guess,
    parse_feedback,
)
from display import (
    colour_feedback,
    print_remaining_info,
    print_colour_legend,
    print_title,
    trigger_easter_egg,
    print_bug_report,
    update_stats,
    print_stats,
)

# HERE LIE THE GAME MODES

def manual_solver(words):
    """
    Interactive mode -- script guesses, human gives feedback.
    ROS2 note: input() -> subscriber callback, print() -> publisher.
    """
    candidates = words[:]
    attempt    = 1

    print("\n  Think of a 5-letter secret word. DO NOT type it.")
    print("  The script will guess. You provide the feedback.\n")
    print_colour_legend()
    print("  Accepted formats:  GGGGG  |  G G G G G  |  g,b,i,g,g  |  gBiGg  etc.\n")

    while True:
        if not candidates:
            print("\n  No candidates left. Your feedback may have been inconsistent.")
            return

        guess = choose_opening_guess(words) if attempt == 1 else choose_best_guess(candidates)

        print(f"\n  ── Attempt {attempt} ──")
        print(f"  Guess: {guess.upper()}")

        try:
            feedback_input = input("\n  Enter 5 results: ").strip()
            feedback       = parse_feedback(feedback_input)
        except ValueError as e:
            print(f"\n  Error: {e}  -> Try again.")
            continue

        print(colour_feedback(guess, feedback))

        if all(f == GOOD for f in feedback):
            print(f"\n  Solved in {attempt} attempt(s)! The word was: {guess.upper()}")
            update_stats(attempt)
            return

        candidates = filter_candidates(candidates, guess, feedback)
        print_remaining_info(candidates)
        attempt += 1


def auto_test_solver(words):
    """
    Auto mode -- script plays against a known target.
    Triggers easter egg if the word is embarrassingly easy.
    """
    target = input("\n  Enter the hidden 5-letter word for testing: ").strip().lower()

    if not target.isalpha() or len(target) != 5:
        print("  Please enter exactly 5 letters.")
        return

    if target not in words:
        print("  That word is not in dictionary.txt")
        return

    if target in EASTER_EGG_WORDS:
        trigger_easter_egg(target)

    candidates = words[:]
    attempt    = 1

    print(f"\n  Testing solver against: {target.upper()}\n")

    while True:
        if not candidates:
            print("  No candidates left -- something went wrong.")
            return

        guess    = choose_opening_guess(words) if attempt == 1 else choose_best_guess(candidates)
        feedback = score_guess_against_target(guess, target)

        print(f"  Attempt {attempt}: {guess.upper()}  ->  {colour_feedback(guess, feedback).strip()}")

        if guess == target:
            print(f"\n  Solved in {attempt} attempt(s)!")
            update_stats(attempt)
            return

        candidates = filter_candidates(candidates, guess, feedback)
        print_remaining_info(candidates)
        attempt += 1

def main():
    words = load_dictionary("dictionary.txt")

    while True:
        print_title()
        print("  1 = Manual mode  (you think of a word, script guesses)")
        print("  2 = Auto test    (script plays against a known word)")
        print("  3 = Stats        (session performance)")
        print("  4 = Quit")
        print("  5 = Bug\n")

        mode = input("  Choose mode: ").strip()

        if mode == "1":
            manual_solver(words)
        elif mode == "2":
            auto_test_solver(words)
        elif mode == "3":
            print_stats()
        elif mode == "4":
            print_stats()
            print("\n  Goodbye.\n")
            break
        elif mode == "5":
            print_bug_report()
        else:
            print("  Invalid choice. Pick 1, 2, 3, 4, or 5.")


if __name__ == "__main__":
    main()
