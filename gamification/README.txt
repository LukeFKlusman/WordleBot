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

ROS2 PACKAGES:
  - rclpy
  - std_msgs
  - std_srvs

PYTHON PACKAGES (all standard, no pip install needed):
  - os, sys, json (built-in)

INTERNAL FILES (must be in same folder as gamification_node.py):
  - wordle_logic.py
  - dictionary.py
  - constants.py
  - dictionary.txt

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

You should see:
   [INFO] Dictionary loaded: 14855 words
   [INFO] Gamification node ready.
   [INFO] Subscribing: /perception/detections ...
   [INFO] Publishing:  /gamification/guess ...

The node is now running and waiting for messages.

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
  Format  : JSON — {"blocks": [{"letter":"A","conf":94.2,"x_m":0.04,"y_m":-0.02,"z_m":0.38,"theta_deg":12.5}]}
  Purpose : Receives detected letter blocks from the RealSense camera CNN node.
            Each block contains the detected letter, confidence score, and 3D
            position in camera frame. The node stores all letters and positions,
            filters candidates to only formable words, then picks the best guess.
  Note    : x_m/y_m/z_m are in camera frame. SS2 applies EE-to-robot transform.

/mission/state
  Type    : std_msgs/String
  From    : Subsystem 3 — Interaction and Execution (Elijah)
  Values  : START | RESUME | SCANNING | STOP | IDLE | RESET
  Purpose : Controls game lifecycle. START begins the game, STOP pauses it,
            RESET wipes everything and unlocks the mode so a new game can begin.
  Note    : START is ignored if no game mode has been selected first.

/gamification/feedback
  Type    : std_msgs/String
  From    : Subsystem 3 GUI or voice node
  Format  : Five space-separated characters — e.g. "G B I G G" or "GBIG G"
            G = Good (correct position)
            B = Bad position (letter exists, wrong spot)
            I = Incorrect (letter not in word)
  Purpose : After the robot places letters on the board, the human evaluates
            the result and sends feedback. The node uses this to filter the
            candidate list and pick the next guess.
  Note    : Only accepted in Mode A. Ignored in Mode B.

/gamification/mode
  Type    : std_msgs/String
  From    : Subsystem 3 GUI
  Values  : MODE_A | MODE_B
  Purpose : Selects the game mode. Locks in once selected — cannot change
            mid-game. Must reset before switching modes.

/gamification/secret_word
  Type    : std_msgs/String
  From    : Subsystem 3 GUI
  Format  : Plain 5-letter string e.g. "crane"
  Purpose : In Mode A, the human sets the secret word the robot will try to
            guess. Must be a valid word in dictionary.txt. Sending this
            automatically starts the game and triggers the first guess.

/gamification/player_guess
  Type    : std_msgs/String
  From    : Subsystem 3 GUI
  Format  : Plain 5-letter string e.g. "crane"
  Purpose : In Mode B, the human submits their guess. The node scores it
            against the hidden secret word and publishes G/B/I feedback
            to /diagnostics.

--------------------------------------------------------------------------------
  TOPICS PUBLISHED
--------------------------------------------------------------------------------

/gamification/guess
  Type    : std_msgs/String
  To      : Subsystem 3 GUI (Elijah)
  Format  : Plain uppercase 5-letter word e.g. "CRANE"
  Purpose : The current word guess. GUI displays this on the Wordle board
            via previewGuess() and submitPreviewGuess().
  QoS     : Default

/gamification/mission_state
  Type    : std_msgs/String
  To      : Subsystem 2 — Motion Planning and Control (Connor)
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
            ACTIVE          — game is running normally
            GUESSING        — node has published a guess, waiting for feedback
            WAITING_FOR_LETTERS — no formable words from detected letters yet
            SELECT_MODE     — start attempted before mode was chosen
            SELECT_SECRET   — Mode A selected, waiting for secret word
            PAUSED          — game stopped via STOP signal
            SOLVED          — word guessed correctly
            GAME_OVER       — reached max attempts without solving
            RESET           — game has been reset
            ERROR           — no candidates left or inconsistent feedback
            INVALID_GUESS   — player guess not in dictionary (Mode B)
            INVALID_SECRET  — secret word not in dictionary (Mode A)
            MODE_LOCKED     — tried to change mode mid-game
  QoS     : Default

/hl_control/word_request
  Type    : std_msgs/String
  To      : High Level Controller
  Format  : Plain uppercase 5-letter word e.g. "CRANE"
  Purpose : Sends the confirmed word to the HL controller which automatically
            arms the robot and runs the task sequencer. No separate start
            signal needed — HL control acts as soon as word arrives.
  QoS     : TRANSIENT_LOCAL (latched, depth=1)
            If HL controller starts after this node has already published,
            it will still receive the last word automatically.

--------------------------------------------------------------------------------
  SERVICES
--------------------------------------------------------------------------------

/gamification/reset
  Type    : std_srvs/Trigger
  Purpose : Full game reset. Clears all state including mode lock, candidates,
            current guess, attempt counter, and feedback history.
  Call    : ros2 service call /gamification/reset std_srvs/srv/Trigger

/gamification/undo
  Type    : std_srvs/Trigger
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
10. Receives feedback, filters candidates, sets guess_pending = True
11. Loops back to step 6 for the next attempt
12. If all feedback is GOOD → publishes SOLVED, game ends
13. If attempt reaches 6 → publishes GAME_OVER, game ends

--------------------------------------------------------------------------------
  EXPECTED ERRORS AND HOW TO FIX THEM
--------------------------------------------------------------------------------

ERROR: ModuleNotFoundError: No module named 'rclpy'
FIX:   ROS2 is not sourced. Run:
       source /opt/ros/humble/setup.bash

ERROR: FileNotFoundError: dictionary.txt not found
FIX:   Run the node from the gamification folder:
       cd ~/RS2/gamification && python3 gamification_node.py

ERROR: ModuleNotFoundError: No module named 'wordle_logic'
FIX:   Run from the correct directory — gamification_node.py uses sys.path.append
       to find sibling files. Must be run from inside the gamification folder.

ERROR: Node starts but nothing happens
EXPECTED — the node waits for a mode selection and start signal before doing
anything. This is intentional. Send MODE_A or MODE_B to begin.

ERROR: "Start ignored because no game mode is selected"
FIX:   Send mode selection before sending START:
       ros2 topic pub --once /gamification/mode std_msgs/String "data: 'MODE_A'"

ERROR: "Reset the game before changing modes"
FIX:   Mode is locked once selected. Send RESET first:
       ros2 topic pub --once /mission/state std_msgs/String "data: 'RESET'"

ERROR: x_m/y_m/z_m are null in mission_state
EXPECTED — this happens when testing without perception running. The Wordle
logic still works correctly. Null positions mean SS2 cannot execute physical
placement but the word solver itself is functioning normally.

ERROR: Waiting for at least 1 matching subscription
FIX:   The node is not running. Start gamification_node.py first, then send
       topic commands from a second terminal.

ERROR: No candidates left — feedback may be inconsistent
FIX:   Feedback entered incorrectly. Reset the game and start again:
       ros2 topic pub --once /mission/state std_msgs/String "data: 'RESET'"

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

Check topic info:
   ros2 topic info /gamification/guess

Call reset service:
   ros2 service call /gamification/reset std_srvs/srv/Trigger

Call undo service:
   ros2 service call /gamification/undo std_srvs/srv/Trigger

Check node info:
   ros2 node info /gamification_node

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
  Each placement includes letter, position index, and x/y/z in camera frame

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