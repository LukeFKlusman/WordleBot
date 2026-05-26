document.addEventListener("DOMContentLoaded", () => {
  const board = document.getElementById("board");
  const statusBadge = document.getElementById("status-badge");
  const statusText = document.getElementById("status-text");
  const modeAButton = document.getElementById("mode-a-button");
  const modeBButton = document.getElementById("mode-b-button");
  const secretForm = document.getElementById("secret-form");
  const secretInput = document.getElementById("secret-input");
  const resetGameButton = document.getElementById("reset-game-button");

  const TILE_COUNT = 30;
  const ROW_LENGTH = 5;
  const FEEDBACK_STATES = [
    { key: "unset", token: "", className: "state-unset" },
    { key: "incorrect", token: "I", className: "state-incorrect" },
    { key: "bad-position", token: "B", className: "state-bad-position" },
    { key: "good", token: "G", className: "state-good" }
  ];

  let activeRowIndex = 0;
  let lastDiagnostics = {};
  let gameMode = null;
  let modeLocked = false;
  let lastAppliedFeedbackKey = "";
  let lastTerminalPromptKey = "";

  function createSquares() {
    for (let index = 0; index < TILE_COUNT; index += 1) {
      const square = document.createElement("button");
      square.type = "button";
      square.classList.add("square", "state-unset");
      square.dataset.row = String(Math.floor(index / ROW_LENGTH));
      square.dataset.col = String(index % ROW_LENGTH);
      square.dataset.feedbackIndex = "0";
      square.setAttribute("aria-label", `Tile ${index + 1}`);
      square.addEventListener("click", () => cycleTile(square));
      board.appendChild(square);
    }
  }

  function rowTiles(rowIndex) {
    return Array.from(board.children).slice(
      rowIndex * ROW_LENGTH,
      (rowIndex + 1) * ROW_LENGTH
    );
  }

  function clampRowIndex(rowIndex) {
    return Math.max(0, Math.min(rowIndex, TILE_COUNT / ROW_LENGTH - 1));
  }

  function setTileState(tile, stateIndex) {
    FEEDBACK_STATES.forEach((state) => tile.classList.remove(state.className));
    tile.dataset.feedbackIndex = String(stateIndex);
    tile.classList.add(FEEDBACK_STATES[stateIndex].className);
  }

  function cycleTile(tile) {
    if (!modeLocked || gameMode !== "A") {
      return;
    }

    if (Number(tile.dataset.row) !== activeRowIndex || !tile.textContent) {
      return;
    }

    const nextIndex = (Number(tile.dataset.feedbackIndex) + 1) % FEEDBACK_STATES.length;
    setTileState(tile, nextIndex);
  }

  function clearRow(rowIndex) {
    rowTiles(rowIndex).forEach((tile) => {
      tile.textContent = "";
      setTileState(tile, 0);
    });
  }

  function cleanWord(value) {
    return value.toUpperCase().replace(/[^A-Z]/g, "").slice(0, ROW_LENGTH);
  }

  function focusGame() {
    window.focus();
    document.body.focus({ preventScroll: true });
  }

  function rowFeedback(rowIndex) {
    return rowTiles(rowIndex)
      .map((tile) => FEEDBACK_STATES[Number(tile.dataset.feedbackIndex)].token)
      .join("");
  }

  function clearBoard() {
    for (let rowIndex = 0; rowIndex < TILE_COUNT / ROW_LENGTH; rowIndex += 1) {
      clearRow(rowIndex);
    }
    activeRowIndex = 0;
    lastAppliedFeedbackKey = "";
    lastTerminalPromptKey = "";
  }

  function setMode(mode, publishChange = false, locked = modeLocked) {
    if (publishChange && modeLocked) {
      statusBadge.textContent = "MODE LOCKED";
      return;
    }

    gameMode = mode === "A" || mode === "B" ? mode : null;
    modeLocked = Boolean((publishChange || locked) && gameMode);
    modeAButton.classList.toggle("active", gameMode === "A");
    modeBButton.classList.toggle("active", gameMode === "B");
    modeAButton.disabled = modeLocked;
    modeBButton.disabled = modeLocked;
    secretForm.classList.toggle("hidden", !(modeLocked && gameMode === "A"));
    updateStatusFromDiagnostics();

    if (publishChange && gameMode) {
      document.title = `wordle_mode:${gameMode}`;
    }
  }

  function resetGame(publishChange = false) {
    clearBoard();
    gameMode = null;
    modeLocked = false;
    lastDiagnostics = {};
    secretInput.value = "";
    setMode(null, false, false);
    statusBadge.textContent = "SELECT MODE";
    statusText.textContent = "Pick a mode";

    if (publishChange) {
      document.title = "reset_game";
    }
  }

  function updateStatusFromDiagnostics() {
    if (!modeLocked) {
      statusBadge.textContent = "SELECT MODE";
      statusText.textContent = "Pick a mode";
        return;
    }

    const status = lastDiagnostics.status || "READY";
    const attempt = lastDiagnostics.attempt || activeRowIndex + 1;
    const candidates = lastDiagnostics.candidates_left;

    if (status === "GAME_OVER") {
      statusBadge.textContent = "LOST";
    } else {
      statusBadge.textContent = status;
    }
    statusText.textContent = `Attempt ${attempt}`;

    if (isTerminalStatus(status)) {
      const solution = cleanWord(lastDiagnostics.solution_word || "");
      if (status === "SOLVED") {
      } else if (solution) {
      } else {
      }
      return;
    }

    if (typeof candidates === "number") {
    } else {
    }
  }

  function setGuessForRow(rowIndex, guess) {
    const cleaned = cleanWord(guess);
    if (cleaned.length !== ROW_LENGTH) {
      return;
    }

    const targetRowIndex = clampRowIndex(rowIndex);
    const tiles = rowTiles(targetRowIndex);
    tiles.forEach((tile, index) => {
      tile.textContent = cleaned[index] || "";
      setTileState(tile, 0);
    });

    const attempt = Number(lastDiagnostics.attempt) || targetRowIndex + 1;
    statusText.textContent = `Attempt ${attempt} | Guess ${cleaned}`;
  }

  function setActiveGuess(guess) {
    if (!modeLocked) {
      return;
    }

    if (activeRowIndex >= TILE_COUNT / ROW_LENGTH) {
      activeRowIndex = TILE_COUNT / ROW_LENGTH - 1;
      clearRow(activeRowIndex);
    }

    setGuessForRow(activeRowIndex, guess);
  }

  function clearActiveGuess() {
    if (!modeLocked) {
      return;
    }

    clearRow(activeRowIndex);
  }

  function isTerminalStatus(status) {
    return status === "SOLVED" || status === "GAME_OVER";
  }

  function submitActiveGuess() {
    if (!modeLocked) {
      statusBadge.textContent = "SELECT MODE";
      return;
    }

    if (isTerminalStatus(lastDiagnostics.status)) {
      statusBadge.textContent = "RESET";
      return;
    }

    const tiles = rowTiles(activeRowIndex);
    const guess = tiles.map((tile) => tile.textContent).join("");

    if (gameMode === "B") {
      if (guess.length !== ROW_LENGTH) {
        statusBadge.textContent = "INCOMPLETE";
        return;
      }

      document.title = `player_guess:${guess}`;
      return;
    }

    const feedback = rowFeedback(activeRowIndex);
    const hasUnsetTile = tiles.some(
      (tile) => !FEEDBACK_STATES[Number(tile.dataset.feedbackIndex)].token
    );

    if (guess.length !== ROW_LENGTH) {
      statusBadge.textContent = "INCOMPLETE";
      return;
    }

    if (feedback.length !== ROW_LENGTH || hasUnsetTile) {
      statusBadge.textContent = "SET FEEDBACK";
      return;
    }

    document.title = `feedback:${feedback}`;
    activeRowIndex = Math.min(activeRowIndex + 1, TILE_COUNT / ROW_LENGTH - 1);
    updateStatusFromDiagnostics();
  }

  function handleKeyInput(input) {
    if (input === "enter") {
      submitActiveGuess();
      return;
    }

    if (input === "backspace") {
      if (modeLocked && gameMode === "B") {
        const tiles = rowTiles(activeRowIndex);
        const filledTiles = tiles.filter((tile) => tile.textContent);
        if (filledTiles.length > 0) {
          filledTiles[filledTiles.length - 1].textContent = "";
        }
        return;
      }

      clearActiveGuess();
      return;
    }

    if (modeLocked && gameMode === "B" && /^[a-z]$/.test(input)) {
      const tiles = rowTiles(activeRowIndex);
      const nextTile = tiles.find((tile) => !tile.textContent);
      if (nextTile) {
        nextTile.textContent = input.toUpperCase();
        setTileState(nextTile, 0);
      }
    }
  }

  function applyFeedbackForModeB() {
    if (gameMode !== "B") {
      return;
    }

    const guess = cleanWord(lastDiagnostics.current_guess || "");
    const feedback = (lastDiagnostics.last_feedback || "").toUpperCase();
    const scoredAttempt = Number(lastDiagnostics.scored_attempt);
    if (guess.length !== ROW_LENGTH || feedback.length !== ROW_LENGTH || !scoredAttempt) {
      return;
    }

    const key = `${scoredAttempt}:${guess}:${feedback}`;
    if (key === lastAppliedFeedbackKey) {
      return;
    }
    lastAppliedFeedbackKey = key;

    const rowIndex = clampRowIndex(scoredAttempt - 1);
    setGuessForRow(rowIndex, guess);
    rowTiles(rowIndex).forEach((tile, index) => {
      const stateIndex = FEEDBACK_STATES.findIndex((state) => state.token === feedback[index]);
      setTileState(tile, Math.max(stateIndex, 0));
    });

    activeRowIndex = clampRowIndex(rowIndex + 1);
  }

  function promptForRestartIfTerminal() {
    const status = lastDiagnostics.status || "";
    if (!isTerminalStatus(status)) {
      return;
    }

    const attempt = Number(lastDiagnostics.scored_attempt || lastDiagnostics.attempt || 0);
    const solution = cleanWord(lastDiagnostics.solution_word || "");
    const key = `${status}:${attempt}:${solution}`;
    if (key === lastTerminalPromptKey) {
      return;
    }
    lastTerminalPromptKey = key;

    const message = status === "SOLVED"
      ? `${gameMode === "A" ? "AI solved the word" : "You solved the word"}. Start a new game?`
      : `${gameMode === "A" ? "AI lost" : "You lost"}.${solution ? ` The solution was ${solution}.` : ""} Start a new game?`;

    window.setTimeout(() => {
      if (window.confirm(message)) {
        resetGame(true);
      }
    }, 0);
  }

  function setDiagnostics(rawPayload) {
    try {
      lastDiagnostics = JSON.parse(rawPayload);
    } catch (_error) {
      lastDiagnostics = {};
    }

    const nextModeLocked = Boolean(lastDiagnostics.mode_locked);
    if (!nextModeLocked || lastDiagnostics.status === "RESET") {
      resetGame(false);
    } else {
      setMode(lastDiagnostics.mode || gameMode, false, nextModeLocked);
    }
    applyFeedbackForModeB();
    updateStatusFromDiagnostics();
    promptForRestartIfTerminal();
  }

  function resetQtTitle() {
    document.title = "Wordle Operator Console";
  }

  function isEditableTarget(target) {
    if (!target) {
      return false;
    }

    const tagName = target.tagName;
    return target.isContentEditable || tagName === "INPUT" || tagName === "TEXTAREA" ||
      tagName === "SELECT" || tagName === "BUTTON";
  }

  window.wordleHandleQtKey = handleKeyInput;
  window.wordleSetQtGuess = setActiveGuess;
  window.wordleSetActiveGuess = setActiveGuess;
  window.wordleClearQtGuess = clearActiveGuess;
  window.wordleSubmitQtGuess = submitActiveGuess;
  window.wordleSetDiagnostics = setDiagnostics;
  window.wordleSetMode = (mode) => setMode(mode, false);
  window.wordleResetQtTitle = resetQtTitle;

  modeAButton.addEventListener("click", () => setMode("A", true));
  modeBButton.addEventListener("click", () => setMode("B", true));
  resetGameButton.addEventListener("click", () => resetGame(true));
  secretForm.addEventListener("submit", (event) => {
    event.preventDefault();
    const cleaned = cleanWord(secretInput.value);
    if (cleaned.length !== ROW_LENGTH) {
      statusBadge.textContent = "INVALID";
      return;
    }
    secretInput.value = cleaned;
    document.title = `secret_word:${cleaned}`;
  });

  createSquares();
  resetGame(false);
  resetQtTitle();
  document.body.tabIndex = -1;
  focusGame();
  document.addEventListener("pointerdown", focusGame);
  document.addEventListener("keydown", (event) => {
    if (isEditableTarget(event.target)) {
      return;
    }

    handleKeyInput(event.key.toLowerCase());
  });
});
