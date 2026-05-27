```bash

ros2 topic pub --once /hl_control/word_request std_msgs/msg/String "{data: 'CRANE'}"

ros2 topic pub --once /perception/gameboard_state hl_control/msg/GameboardState \
  "{letters: [
    {letter: 'A', object_id: 'A_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.30, y: 0.15, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.247, w: 0.969}}}},
    {letter: 'R', object_id: 'R_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.30, y: 0.30, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.565, w: 0.825}}}},
    {letter: 'B', object_id: 'B_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.30, y: 0.225, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.867, w: 0.498}}}},
    {letter: 'C', object_id: 'C_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.075, y: 0.375, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: -0.389, w: 0.921}}}},
    {letter: 'N', object_id: 'N_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.30, y: 0.375, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.867, w: 0.498}}}},
    {letter: 'X', object_id: 'X_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.30, y: 0.375, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.867, w: 0.498}}}},
    {letter: 'E', object_id: 'E_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.30, y: 0.150, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: -0.682, w: 0.732}}}},
    {letter: 'W', object_id: 'W_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.15, y: 0.225, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}},
    {letter: 'D', object_id: 'D_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.0, y: 0.225, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}

  ]}"


ros2 topic pub --once /perception/gameboard_state hl_control/msg/GameboardState \
  "{letters: [
    {letter: 'C', object_id: 'C_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.448, y: 0.072, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'R', object_id: 'R_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.377, y: 0.154, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'A', object_id: 'A_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.302, y: 0.302, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'N', object_id: 'N_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.373, y: 0.220, z: 0.025},
                   orientation: {w: 1.0}}}},
    {letter: 'E', object_id: 'E_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.157, y: 0.304, z: 0.025},
                   orientation: {w: 1.0}}}}
  ]}"

ros2 topic pub --once /perception/gameboard_state hl_control/msg/GameboardState \
  "{letters: [
    {letter: 'T', object_id: 'T_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.30, y: 0.15, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}},
    {letter: 'R', object_id: 'R_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.30, y: 0.30, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}},
    {letter: 'A', object_id: 'A_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.075, y: 0.375, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}},
    {letter: 'I', object_id: 'I_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.30, y: 0.375, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}},
    {letter: 'E', object_id: 'E_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.30, y: 0.150, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}
  ]}"

ros2 topic pub --once /perception/gameboard_state hl_control/msg/GameboardState \
  "{letters: [
    {letter: 'T', object_id: 'T_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.30, y: 0.15, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}},
    {letter: 'S', object_id: 'S_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.30, y: 0.30, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.0 w: 1.0}}}},
    {letter: 'A', object_id: 'A_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.075, y: 0.375, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}},
    {letter: 'L', object_id: 'L_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.30, y: 0.375, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}},
    {letter: 'E', object_id: 'E_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.30, y: 0.150, z: 0.040},
                   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}
  ]}"


  ros2 topic pub --once /perception/gameboard_state hl_control/msg/GameboardState \
  "{letters: [
    {letter: 'C', object_id: 'C_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.30, y: 0.15, z: 0.040},
                   orientation: {w: 1.0}}}},
    {letter: 'G', object_id: 'G_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.30, y: 0.30, z: 0.040},
                   orientation: {w: 1.0}}}},
    {letter: 'A', object_id: 'A_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.075, y: 0.375, z: 0.040},
                   orientation: {w: 1.0}}}},
    {letter: 'R', object_id: 'R_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.30, y: 0.375, z: 0.040},
                   orientation: {w: 1.0}}}},
    {letter: 'E', object_id: 'E_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.30, y: 0.150, z: 0.040},
                   orientation: {w: 1.0}}}},
  ]}"

ros2 topic pub --once /perception/gameboard_state hl_control/msg/GameboardState \
  "{letters: [
    {letter: 'C', object_id: 'C_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.30, y: 0.15, z: 0.040},
                   orientation: {w: 1.0}}}},
    {letter: 'G', object_id: 'G_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.30, y: 0.30, z: 0.040},
                   orientation: {w: 1.0}}}},
    {letter: 'A', object_id: 'A_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.075, y: 0.375, z: 0.040},
                   orientation: {w: 1.0}}}},
    {letter: 'R', object_id: 'R_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.30, y: 0.375, z: 0.040},
                   orientation: {w: 1.0}}}},
    {letter: 'E', object_id: 'E_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.30, y: 0.150, z: 0.040},
                   orientation: {w: 1.0}}}},
  ]}"

  ros2 topic pub --once /perception/gameboard_state hl_control/msg/GameboardState \
  "{letters: [
    {letter: 'C', object_id: 'C_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.30, y: 0.15, z: 0.040},
                   orientation: {w: 1.0}}}},
    {letter: 'N', object_id: 'N_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.30, y: 0.30, z: 0.040},
                   orientation: {w: 1.0}}}},
    {letter: 'A', object_id: 'A_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.075, y: 0.375, z: 0.040},
                   orientation: {w: 1.0}}}},
    {letter: 'R', object_id: 'R_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: -0.30, y: 0.375, z: 0.040},
                   orientation: {w: 1.0}}}},
    {letter: 'E', object_id: 'E_object_1',
     pose: {header: {frame_id: 'world'},
            pose: {position: {x: 0.30, y: 0.150, z: 0.040},
                   orientation: {w: 1.0}}}},
  ]}"


  ```