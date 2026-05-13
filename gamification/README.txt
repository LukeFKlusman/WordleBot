================================================================================
  SUBSYSTEM 4 — GAMIFICATION NODE
  CELK RS2 | UTS FEIT
  Author: Kermit Nicou (14184030)
================================================================================

--------------------------------------------------------------------------------
  OVERVIEW
--------------------------------------------------------------------------------

The Gamification Node is a ROS2 Python node responsible for the Wordle solving
logic of the RS2 robot system. It sits between Subsystem 1 (Perception) and
Subsystem 2 (Motion Planning and Control), receiving detected letter blocks from
the camera, running a frequency-ranked Wordle algorithm to determine the best
possible word, and publishing an ordered letter placement sequence for the robot
arm to execute.

The node supports two game modes:
  Mode A — Robot guesses. Human sets a secret word and provides G/B/I feedback.
  Mode B — Human guesses. Robot picks a secret word and scores each guess.

--------------------------------------------------------------------------------
  CODE FLOW (plain language)
--------------------------------------------------------------------------------

I subscribe to /perception/detections which gives me a JSON string containing
detected letter blocks — each block has a letter, confidence score, and x,y,z
position in camera space. I parse that JSON and store all detected letters into
available_letters and their 3D positions into block_positions. I then filter my
full 14,855 word candidate list down to only words that can be formed from those
detected letters. I run a frequency-ranked scoring algorithm to pick the best
guess, publish that word as a plain string to the GUI, publish an ordered x,y,z
placement sequence for each letter to motion planning, and publish the confirmed
word to the HL controller on a latched topic so it arms the robot automatically
without needing a separate start signal. After the robot places the letters I
receive G/B/I feedback, use it to eliminate any word from the candidate list that
couldn't have produced that result, and repeat the whole process until the word
is solved or 6 attempts are reached.

--------------------------------------------------------------------------------
  FILE STRUCTURE
--------------------------------------------------------------------------------

gamification/
  gamification_node.py      — Main ROS2 node (this file)
  wordle_logic.py           — Core Wordle algorithm (scoring, filtering, guessing)
  dictionary.py             — Word list loader
  constants.py              — Shared constants (GOOD, BAD_POSITION, INCORRECT)
  dictionary.txt            — 14,855 valid 5-letter words
  main.py                   — Standalone terminal version (no ROS2 required)
  display.py                — Terminal display helpers

voice_control/
  main.py                   — Voice-controlled game loop
  speaker_verification.py   — MFCC voiceprint registration and verification
  voiceprint.json           — Saved player voiceprint (auto-generated)

--------------------------------------------------------------------------------
  DEPENDENCIES
--------------------------------------------------------------------------------

SYSTEM:
  - Ubuntu 22.04 (via WSL2 or native)
  - ROS2 Humble Hawksbill
  - Python 3.10+

ROS2 PACKAGES (lines 8-11 in gamification_node.py):
  - rclpy                   — line 8,  core ROS2 Python library
  - rclpy.qos               — line 9,  needed for latched QoS on HL control topic
  - std_msgs.msg.String     — line 10, message type for all topics
  - std_srvs.srv.Trigger    — line 11, service type for reset and undo

PYTHON PACKAGES (all built-in, no pip install needed):
  - os, sys, json

INTERNAL FILES (must be in same folder as gamification_node.py):
  - wordle_logic.py         — line 44, all game logic functions
  - dictionary.py           — line 43, load_dictionary()
  - constants.py            — line 52, GOOD, BAD_POSITION, INCORRECT tokens
  - dictionary.txt          — line 56, loaded at startup

NOTE: sys.path.append on line 41 adds the gamification folder to Python's
import path so sibling files can be found regardless of where the script is
called from. This only works correctly if you run from inside the folder.

OPTIONAL (for voice control):
  pip install SpeechRecognition sounddevice scipy numpy soundfile librosa

--------------------------------------------------------------------------------
  SETUP — EVERY NEW TERMINAL
--------------------------------------------------------------------------------

1. Source ROS2:
   source /opt/ros/humble/setup.bash

2. Set domain ID (isolates our system from other groups in the lab):
   export ROS_DOMAIN_ID=10

3. Navigate to repo:
   cd ~/RS2

--------------------------------------------------------------------------------
  HOW TO RUN
--------------------------------------------------------------------------------

Terminal 1 — Run the gamification node:
   cd ~/RS2/gamification
   export ROS_DOMAIN_ID=10
   python3 gamification_node.py

Entry point is main() at the bottom of the file (line 296).
On startup the node:
  - Runs __init__ (line 54)
  - Loads dictionary.txt into memory (line 56)
  - Initialises candidates to full 14,855 word list (line 62)
  - Sets all game state variables to defaults (lines 62-73)
  - Creates latched QoS profile (lines 76-79)
  - Registers 6 subscribers (lines 82-87)
  - Registers 4 publishers (lines 90-96)
  - Registers 2 services (lines 99-100)
  - Spins and waits for messages

You should see:
   [INFO] Dictionary loaded: 14855 words
   [INFO] Gamification node ready.
   [INFO] Subscribing: /perception/detections ...
   [INFO] Publishing:  /gamification/guess ...

IMPORTANT: The node does nothing after starting. game_active = False (line 66)
and every callback checks this flag first. The node waits until a mode is
selected and a game is started before doing anything.

--------------------------------------------------------------------------------
  HOW TO PLAY — MODE A (Robot Guesses)
--------------------------------------------------------------------------------

Open a second terminal, then run each command one at a time.
Remember to set ROS_DOMAIN_ID=10 in every new terminal.

Step 1 — Select Mode A:
   ros2 topic pub --once /gamification/mode std_msgs/String "data: 'MODE_A'"

Step 2 — Set the secret word (the word the robot will try to guess):
   ros2 topic pub --once /gamification/secret_word std_msgs/String "data: 'crane'"

   The node will immediately publish its first guess. Check Terminal 1 to see
   what word it guessed (e.g. SNARE).

Step 3 — Send feedback for each guess:
   Feedback format: G = correct position, B = wrong position, I = not in word
   One character per letter, space separated.

   Example — if the guess was SNARE and secret is CRANE:
     S = not in word     → I
     N = not in word     → I
     A = correct spot    → G
     R = wrong position  → B
     E = wrong position  → B

   ros2 topic pub --once /gamification/feedback std_msgs/String "data: 'I I G B B'"

Step 4 — Repeat Step 3 until the node solves the word or hits 6 attempts.

To watch the current guess:
   ros2 topic echo /gamification/guess

To watch full game state:
   ros2 topic echo /diagnostics

To reset and play again:
   ros2 topic pub --once /mission/state std_msgs/String "data: 'RESET'"

--------------------------------------------------------------------------------
  HOW TO PLAY — MODE B (Human Guesses)
--------------------------------------------------------------------------------

Step 1 — Select Mode B:
   ros2 topic pub --once /gamification/mode std_msgs/String "data: 'MODE_B'"

   The node automatically picks a hidden secret word.

Step 2 — Submit your guess:
   ros2 topic pub --once /gamification/player_guess std_msgs/String "data: 'crane'"

Step 3 — Check the diagnostics to see your feedback:
   ros2 topic echo /diagnostics

   Look for the "last_feedback" field — G/B/I for each letter of your guess.

Step 4 — Keep guessing until you see status: SOLVED or GAME_OVER.

To reset:
   ros2 topic pub --once /mission/state std_msgs/String "data: 'RESET'"

--------------------------------------------------------------------------------
  TOPICS SUBSCRIBED
--------------------------------------------------------------------------------

/perception/detections
  Type    : std_msgs/String
  From    : Subsystem 1 — Perception (Luke)
  Line    : 82
  Format  : JSON — {"blocks": [{"letter":"A","conf":94.2,"x_m":0.04,"y_m":-0.02,"z_m":0.38,"theta_deg":12.5}]}
  Purpose : Receives detected letter blocks from the RealSense camera CNN node.
            Each block contains the detected letter, confidence score, and 3D
            position in camera frame. The node stores all letters and positions,
            filters candidates to only formable words, then picks the best guess.
            Runs every camera frame but only acts when guess_pending is True.
  Note    : x_m/y_m/z_m are in camera frame. SS2 applies EE-to-robot transform.

/mission/state
  Type    : std_msgs/String
  From    : Subsystem 3 — Interaction and Execution (Elijah)
  Line    : 83
  Values  : START | RESUME | SCANNING | STOP | IDLE | RESET
  Purpose : Controls game lifecycle. START begins the game, STOP pauses it,
            RESET wipes everything and unlocks the mode so a new game can begin.
  Note    : START is silently ignored if no game mode has been selected first (line 140).

/gamification/feedback
  Type    : std_msgs/String
  From    : Subsystem 3 GUI or voice node
  Line    : 84
  Format  : Five space-separated characters — e.g. "G B I G G" or "GBIGG"
            G = Good (correct position)
            B = Bad position (letter exists, wrong spot)
            I = Incorrect (letter not in word)
  Purpose : After the robot places letters on the board, the human evaluates
            the result and sends feedback. The node calls filter_candidates()
            to narrow the word list, increments attempt, sets guess_pending=True,
            and triggers the next guess.
  Note    : Only accepted in Mode A (line 178). Ignored in Mode B.

/gamification/mode
  Type    : std_msgs/String
  From    : Subsystem 3 GUI
  Line    : 85
  Values  : MODE_A | MODE_B
  Purpose : Selects the game mode. Locks in once selected (line 168) — cannot
            change mid-game. Must reset before switching modes.

/gamification/secret_word
  Type    : std_msgs/String
  From    : Subsystem 3 GUI
  Line    : 86
  Format  : Plain 5-letter string e.g. "crane"
  Purpose : In Mode A, the human sets the secret word the robot will try to
            guess. Must be a valid word in dictionary.txt. Sending this
            automatically starts the game and triggers the first guess.

/gamification/player_guess
  Type    : std_msgs/String
  From    : Subsystem 3 GUI
  Line    : 87
  Format  : Plain 5-letter string e.g. "crane"
  Purpose : In Mode B, the human submits their guess. The node scores it
            against the hidden secret word using score_guess_against_target()
            and publishes G/B/I feedback to /diagnostics.

--------------------------------------------------------------------------------
  TOPICS PUBLISHED
--------------------------------------------------------------------------------

/gamification/guess
  Type    : std_msgs/String
  To      : Subsystem 3 GUI (Elijah)
  Line    : 91
  Format  : Plain uppercase 5-letter word e.g. "CRANE"
  Purpose : The current word guess. GUI displays this on the Wordle board
            via previewGuess() and submitPreviewGuess().
  QoS     : Default

/gamification/mission_state
  Type    : std_msgs/String
  To      : Subsystem 2 — Motion Planning and Control (Connor)
  Line    : 92
  Format  : JSON —
            {
              "word": "CRANE",
              "attempt": 1,
              "placements": [
                {"position": 1, "letter": "C", "x_m": 0.04, "y_m": -0.02, "z_m": 0.38, "theta_deg": 0.0},
                {"position": 2, "letter": "R", "x_m": 0.11, "y_m": -0.02, "z_m": 0.38, "theta_deg": 5.2},
                ...
              ]
            }
  Purpose : Ordered letter placement sequence for the robot arm. Each letter
            includes its 3D position from perception so SS2 knows where to
            pick it from.
  Note    : x_m/y_m/z_m will be null if perception is not running (test mode).
  QoS     : Default

/diagnostics
  Type    : std_msgs/String
  To      : Subsystem 3 GUI (Elijah)
  Line    : 93
  Format  : JSON —
            {
              "status": "GUESSING",
              "mode": "A",
              "mode_locked": true,
              "attempt": 2,
              "current_guess": "CRANE",
              "candidates_left": 84,
              "available_letters": ["C","R","A","N","E"],
              "top_candidates": ["CRANE","STARE","TRACE"],
              "last_feedback": "IIGBB",
              "scored_attempt": 1,
              "secret_word_set": true,
              "solution_word": null,
              "game_active": true,
              "max_attempts": 6,
              "error": null
            }
  Purpose : Full game state published after every action. GUI uses this to
            update the board display, show remaining attempts, and handle
            error states.
  Status values:
            ACTIVE              — game is running normally
            GUESSING            — node has published a guess, waiting for feedback
            WAITING_FOR_LETTERS — no formable words from detected letters yet
            SELECT_MODE         — start attempted before mode was chosen
            SELECT_SECRET       — Mode A selected, waiting for secret word
            PAUSED              — game stopped via STOP signal
            SOLVED              — word guessed correctly
            GAME_OVER           — reached max attempts without solving
            RESET               — game has been reset
            ERROR               — no candidates left or inconsistent feedback
            INVALID_GUESS       — player guess not in dictionary (Mode B)
            INVALID_SECRET      — secret word not in dictionary (Mode A)
            MODE_LOCKED         — tried to change mode mid-game
  QoS     : Default

/hl_control/word_request
  Type    : std_msgs/String
  To      : High Level Controller
  Line    : 96
  Format  : Plain uppercase 5-letter word e.g. "CRANE"
  Purpose : Sends the confirmed word to the HL controller which automatically
            arms the robot and runs the task sequencer. No separate start
            signal needed — HL control acts as soon as word arrives.
  QoS     : TRANSIENT_LOCAL (latched, depth=1) — lines 76-79
            If HL controller starts after this node has already published,
            it will still receive the last word automatically without resend.

--------------------------------------------------------------------------------
  SERVICES
--------------------------------------------------------------------------------

/gamification/reset
  Type    : std_srvs/Trigger
  Line    : 99
  Purpose : Full game reset. Clears all state including mode lock, candidates,
            current guess, attempt counter, and feedback history.
  Call    : ros2 service call /gamification/reset std_srvs/srv/Trigger

/gamification/undo
  Type    : std_srvs/Trigger
  Line    : 100
  Purpose : One-step undo. Decrements attempt counter by 1 and resets candidate
            list to full word list. Does not restore previous feedback state.
  Call    : ros2 service call /gamification/undo std_srvs/srv/Trigger

--------------------------------------------------------------------------------
  PROCESS FLOW
--------------------------------------------------------------------------------

1.  Node starts — loads dictionary.txt into memory as full candidate list (14,855 words)
2.  Waits for mode selection on /gamification/mode (MODE_A or MODE_B)
3.  Waits for start signal — either secret word (Mode A) or START (Mode B)
4.  Receives detected letter blocks from perception on /perception/detections
5.  Parses JSON and builds available_letters list and block_positions dictionary
6.  Filters full candidate list to only words formable from detected letters
7.  Runs frequency-ranked algorithm to pick best guess from remaining candidates
8.  Publishes guess to GUI, placement sequence to motion planning, word to HL control
9.  Sets guess_pending = False and waits for G/B/I feedback
10. Receives feedback, calls filter_candidates(), sets guess_pending = True
11. Loops back to step 6 for the next attempt
12. If all feedback is GOOD → publishes SOLVED, game ends
13. If attempt reaches 6 → publishes GAME_OVER with solution_word revealed

--------------------------------------------------------------------------------
  KEY LOGIC — LIKELY VIVA QUESTIONS
--------------------------------------------------------------------------------

WHY guess_pending?
  Line 67 — perception publishes at 30fps. Without this flag the node would
  publish a new guess every single frame. guess_pending is set False after
  publishing (line 134) and set back True only after valid feedback is
  received (line 203).

WHY latched QoS on /hl_control/word_request?
  Lines 76-79 — DurabilityPolicy.TRANSIENT_LOCAL. If the HL controller starts
  after gamification has already published the word, it still receives it
  automatically. Without latching it would miss it and the robot would never
  start the pick and place sequence.

HOW does candidate filtering work?
  Line 196 — filter_candidates() from wordle_logic.py. After each G/B/I feedback,
  it removes every word from the candidate list that couldn't have produced that
  exact feedback pattern if it were the secret word. The list shrinks each turn
  until ideally one word remains.

WHAT is Mode A vs Mode B?
  Mode A (line 144) — robot guesses, human has the secret word, human gives
  G/B/I feedback after each placement.
  Mode B (line 151) — human guesses, robot picks the secret word randomly,
  robot scores each human guess automatically using score_guess_against_target().

WHAT happens at MAX_ATTEMPTS = 6?
  Line 189 — if attempt reaches 6 without solving, GAME_OVER is published to
  diagnostics with solution_word revealed in the JSON, and game_active set False.

WHAT is _publish_next_mode_a_guess()?
  Lines 247-278 — called after feedback is received instead of waiting for the
  next perception frame. Checks if detected letters can form any candidate words,
  picks best guess, publishes to all three topics. Falls back to full candidate
  list if detected letters can't form any remaining candidates so the game never
  gets stuck waiting for perception.

WHAT does _reset_game(preserve_mode=False) do?
  Lines 280-293 — clears every piece of state back to defaults. preserve_mode=False
  also unlocks the mode so a completely fresh game can be started with a new mode.

--------------------------------------------------------------------------------
  EXPECTED ERRORS AND HOW TO FIX THEM
--------------------------------------------------------------------------------

ERROR: ModuleNotFoundError: No module named 'rclpy'
CAUSE: ROS2 is not installed or not sourced.
FIX:   source /opt/ros/humble/setup.bash
       export ROS_DOMAIN_ID=10

ERROR: FileNotFoundError: dictionary.txt not found
CAUSE: Node is not being run from the gamification folder.
       load_dictionary() on line 56 builds path relative to the script location.
FIX:   cd ~/RS2/gamification && python3 gamification_node.py

ERROR: ModuleNotFoundError: No module named 'wordle_logic'
CAUSE: sys.path.append on line 41 only adds the correct path when run directly.
FIX:   Always run as: python3 gamification_node.py from inside gamification folder.

ERROR: Node starts but nothing happens
EXPECTED — game_active = False (line 66), all callbacks guard against this.
FIX:   Send mode selection then secret word or START signal.

ERROR: "Start ignored because no game mode is selected"
CAUSE: mission_callback (line 140) checks mode_locked before allowing START.
FIX:   ros2 topic pub --once /gamification/mode std_msgs/String "data: 'MODE_A'"

ERROR: "Reset the game before changing modes"
CAUSE: mode_callback (line 168) rejects mode changes when mode_locked = True.
FIX:   ros2 topic pub --once /mission/state std_msgs/String "data: 'RESET'"

ERROR: x_m/y_m/z_m are null in mission_state
EXPECTED — block_positions.get(letter, {}) on line 237 returns empty dict if
perception is not running. Wordle logic still works. Only physical placement fails.

ERROR: "Waiting for at least 1 matching subscription"
CAUSE: The gamification node is not running.
FIX:   Start gamification_node.py first, then send topic commands.

ERROR: "No formable words from detected letters"
CAUSE: Line 122 — none of the remaining candidates can be formed from letters
       currently visible to the camera.
FIX:   Place more letter blocks under the camera.

ERROR: No candidates left — feedback may be inconsistent
CAUSE: filter_candidates() eliminated everything — feedback was wrong.
FIX:   ros2 topic pub --once /mission/state std_msgs/String "data: 'RESET'"

ERROR: "Ignoring manual feedback because Mode A is not active"
CAUSE: feedback_callback (line 178) rejects feedback when not in Mode A.
FIX:   Make sure MODE_A is selected before sending feedback.

--------------------------------------------------------------------------------
  USEFUL ROS2 COMMANDS
--------------------------------------------------------------------------------

List all active nodes:
   ros2 node list

List all active topics:
   ros2 topic list

Echo a topic (live):
   ros2 topic echo /diagnostics
   ros2 topic echo /gamification/guess
   ros2 topic echo /perception/detections

Check topic info:
   ros2 topic info /gamification/guess

Call reset service:
   ros2 service call /gamification/reset std_srvs/srv/Trigger

Call undo service:
   ros2 service call /gamification/undo std_srvs/srv/Trigger

Check node info:
   ros2 node info /gamification_node

Check your network IP:
   hostname -I

--------------------------------------------------------------------------------
  INTEGRATION NOTES FOR OTHER SUBSYSTEMS
--------------------------------------------------------------------------------

FOR SS1 (Luke — Perception):
  Publish to /perception/detections as std_msgs/String
  Format: {"blocks": [{"letter":"A","conf":94.2,"x_m":0.04,"y_m":-0.02,"z_m":0.38,"theta_deg":0.0}]}
  Only publish when at_position = True (spacebar triggered)

FOR SS2 (Connor — Motion Planning):
  Subscribe to /gamification/mission_state for ordered placement sequence
  Subscribe to /hl_control/word_request for the confirmed word (latched)
  Each placement includes letter, position index 1-5, and x/y/z in camera frame

FOR SS3 (Elijah — Interaction and Execution):
  Subscribe to /gamification/guess to call previewGuess() on the Wordle board
  Subscribe to /diagnostics for full game state
  Publish to /gamification/mode to set MODE_A or MODE_B
  Publish to /gamification/feedback with G/B/I string after robot places letters
  Publish to /mission/state with START/STOP/RESET to control game lifecycle
  START button  → publish "START" to /mission/state
  STOP button   → publish "STOP" to /mission/state
  RESUME button → publish "RESUME" to /mission/state

FOR HL CONTROLLER:
  Subscribe to /hl_control/word_request as std_msgs/String with TRANSIENT_LOCAL QoS
  Receives plain uppercase 5-letter word e.g. "CRANE"
  Node automatically arms robot and runs task sequencer on receipt
  No separate start signal needed

--------------------------------------------------------------------------------
  ROS2 DOMAIN
--------------------------------------------------------------------------------

All nodes in the RS2 system run on ROS_DOMAIN_ID=10
This isolates our system from other groups sharing the same network.
Set this in every terminal before running any ROS2 command:
   export ROS_DOMAIN_ID=10

================================================================================
  END OF README
================================================================================
