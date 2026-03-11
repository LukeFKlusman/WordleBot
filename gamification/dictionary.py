# Edit the dictionary.txt if u want more words
from pathlib import Path


def load_dictionary(file_path="dictionary.txt"):
    """Load and return a sorted list of valid 5-letter words."""
    path = Path(file_path)
    if not path.exists():
        raise FileNotFoundError(
            f"Could not find {file_path}. Make sure it is in the same folder as main.py"
        )
    words = set()
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            word = line.strip().lower()
            if word.isalpha() and len(word) == 5:
                words.add(word)
    return sorted(words)
