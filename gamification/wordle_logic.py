from collections import Counter
import random

from constants import GOOD, BAD_POSITION, INCORRECT, TOP_OPENERS

def score_guess_against_target(guess, target):
    """
    Returns a list of 5 feedback tokens (GOOD, BAD_POSITION, INCORRECT)
    for a given guess scored against the target word.
    """
    result    = [INCORRECT] * 5
    remaining = Counter()

    for i in range(5):
        if guess[i] == target[i]:
            result[i] = GOOD
        else:
            remaining[target[i]] += 1

    for i in range(5):
        if result[i] == GOOD:
            continue
        if remaining[guess[i]] > 0:
            result[i] = BAD_POSITION
            remaining[guess[i]] -= 1

    return result


def word_matches_feedback(candidate, guess, feedback):
    """Returns True if scoring the guess against candidate produces the given feedback."""
    return score_guess_against_target(guess, candidate) == feedback


def filter_candidates(candidates, guess, feedback):
    """Filters candidate list down to words consistent with the feedback received."""
    return [w for w in candidates if word_matches_feedback(w, guess, feedback)]

def choose_opening_guess(words):
    """
    Weighted-random opener from TOP_OPENERS.
    Top word appears ~11% of the time.
    P(same opener twice in a row) ~ 1.6% — well under 50%.
    """
    valid = [(w, wt) for w, wt in TOP_OPENERS.items() if w in words]
    if not valid:
        return random.choice(words)
    openers, weights = zip(*valid)
    return random.choices(openers, weights=weights, k=1)[0]


def choose_best_guess(candidates, top_n=5):
    """
    Scores candidates by letter frequency across remaining pool,
    then randomly picks from top N for variety.
    """
    if not candidates:
        return None
    if len(candidates) <= top_n:
        return random.choice(candidates)
    freq   = Counter(c for word in candidates for c in set(word))
    scored = sorted(candidates, key=lambda w: sum(freq[c] for c in set(w)), reverse=True)
    return random.choice(scored[:top_n])

def parse_feedback(user_input):
    """
    Accepts basically any format the user throws at it:
      GGGGG | G G G G G | g,b,i,g,g | G/B/I/G/G | gBiGg
    Strips all delimiters and reads 5 characters.
    """
    cleaned = (
        user_input.strip()
        .upper()
        .replace(",", "")
        .replace("/", "")
        .replace(".", "")
        .replace("-", "")
        .replace(" ", "")
    )

    if len(cleaned) != 5:
        raise ValueError(
            f"Expected 5 feedback characters, got {len(cleaned)}. "
            "Use G (good), B (bad position), I (incorrect)."
        )

    mapping = {"G": GOOD, "B": BAD_POSITION, "I": INCORRECT}
    parsed  = []

    for ch in cleaned:
        if ch not in mapping:
            raise ValueError(f"Invalid character '{ch}'. Only G, B, and I are allowed.")
        parsed.append(mapping[ch])

    return parsed
