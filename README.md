# 525 Swartdog — Rebuilt Robot Code

This repository contains the robot code for Team 525 (Swartdogs) for the Rebuilt game season.

## Testing and simulation

- Using Epilogue for logging.
- Using AdvantageScope to test values with simulation before in the real world.
- Using PathPlanner and Choreo for pathplanning.


## Dependencies
- Recommended: 
  - [WPILib 2026.2.1 Release](https://github.com/wpilibsuite/allwpilib/releases/tag/v2026.2.1)
- Alternative Minimal Install (Unsupported, lacks vendor dependency management):
  - [Java Development Kit (JDK) 17.](https://adoptium.net/temurin/releases?version=17)
    - Note that the JRE is insufficient; the full JDK is required.
  - [WPILib VsCode Extension](https://marketplace.visualstudio.com/items?itemName=wpilibsuite.vscode-wpilib)
  - (Optional) [Extension Pack for Java](https://marketplace.visualstudio.com/items?itemName=vscjava.vscode-java-pack)

## Workflow

This repo is team-owned, only team members should contribute.
Coordination is done via slack and in-person, alongside the team project board.

1. Create a branch off `main` or the current branch for team focus, as relevant. Name said brach based upon what is being worked on, like ``flywheel-sim`` for flywheel simulation.
2. Implement code or other changes, usually alongside others.
3. Open a pull request and request review from mentors and peers.

Write commits in the format of "if committed this code will do ___". 

## Troubleshooting

- Inspect `robot.log` or console output for stack traces.
- Review WPILib documentation.
- Check WPILib tools to ensure the right dependencies are being used.
- Check with mentors.
- If deploy fails, confirm the RoboRIO and Driver Station are on the same network and that the RoboRIO IP resolves.
