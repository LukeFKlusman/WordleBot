```bash
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

  

  ```