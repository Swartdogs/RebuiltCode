# 525 Swartdog — Rebuilt Robot Code

This repository contains the robot code for Team 525 (Swartdogs) for the Rebuilt game season.

## Testing and simulation

- Using Epilogue for logging.
- Using AdvantageScope to test values with simulation before in the real world.
- Using PathPlanner and Choreo for pathplanning.


## Dependencies

This project depends on WPILib and vendor libraries for motor controllers and sensors. See `build.gradle`/`CMakeLists.txt` for exact dependency versions. For reference and examples, review WPILib: https://github.com/wpilibsuite/allwpilib

## Workflow

This repo is team-owned, only team members should contribute.
Cordination is done via slack and in-person, alongside the team project board.

1. Create a branch off `main` or the current branch in use.
2. Implement code.
3. Open a pull request and request review from mentors and peers.

Write commits in the format of "if committed this code will do ___". 

## Troubleshooting

- If deploy fails, confirm the RoboRIO and Driver Station are on the same network and that the RoboRIO IP resolves.
- Check WPILib tools and driver station versions for compatibility.
- Check with mentors 
- Inspect `robot.log` or console output for stack traces.
- Review WPILib documentation.
