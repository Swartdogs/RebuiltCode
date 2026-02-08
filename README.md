# 525 Swartdog — Rebuilt Robot Code

This repository contains the robot code for Team 525 (Swartdogs) for the Rebuilt game season. This is NOT a reusable library — it is the robot-specific, season-specific code for the single robot built by the team this year.

Reference: The WPILib project for robot libraries and examples: https://github.com/wpilibsuite/allwpilib

## Repository purpose

This repo holds the robot software used during build season, testing, and competition for the Rebuilt season. It includes the robot project, build scripts, and team-specific configuration and utility code.

## Supported hardware

- RoboRIO (official FRC control system)
- Motor controllers: specify Talon SRX / SPARK MAX / Victor SPX as used on the robot (update as appropriate)
- Sensors: IMU/Gyro, encoders, limit switches, ultrasonic/Time-of-Flight (update list to match your robot)
- Pneumatics (PCM) if used

## Quick start

Prerequisites:
- Java 11 or later (or the language/runtime your robot code uses)
- WPILib and vendor libraries installed (see WPILib docs)
- Driver Station software installed

1. Clone the repository:

   git clone https://github.com/Swartdogs/RebuiltCode.git
   cd RebuiltCode

2. Checkout the working branch (example):

   git checkout readme-creation

3. Open the project in your IDE (VS Code or IntelliJ recommended for Java/C++). Use the WPILib extension if available.

4. Build the code and deploy to the RoboRIO using WPILib tools or gradle tasks supplied in the repo. Example (Java/Gradle):

   ./gradlew build
   ./gradlew deploy --project-prop roboRIO=roborio-525-frc.local

Adjust commands for your language or build system if different.

## Project structure (example)

- src/ - Robot source code
- build.gradle / CMakeLists.txt - Build configuration files
- deploy/ - Deployment and packaging scripts (if present)
- docs/ - Team and wiring documentation (keep up to date)
- tests/ - Unit and integration tests

Tailor the above to match the repository contents — update paths and filenames as needed.

## Testing and simulation

- Use WPILib Simulator (if configured) to test robot code off-robot.
- Run unit tests with the build system (e.g., `./gradlew test`).
- For subsystem-level tests, use mocked hardware interfaces where possible.

## Dependencies

This project depends on WPILib and possibly vendor libraries for motor controllers and sensors. See `build.gradle`/`CMakeLists.txt` for exact dependency versions. For reference and examples, review WPILib: https://github.com/wpilibsuite/allwpilib

## Contributing

This repo is team-owned; contributions should follow team processes. Typical workflow:

1. Create a feature branch off `main` or the team branch you use.
2. Implement code, add tests and update documentation.
3. Open a pull request and request review from mentors or captains.

Keep commits focused and include test or deployment instructions when relevant.

## Troubleshooting

- If deploy fails, confirm the RoboRIO and Driver Station are on the same network and that the RoboRIO IP resolves.
- Check WPILib tools and driver station versions for compatibility.
- Inspect `robot.log` or console output for stack traces.

## License and ownership

This repository contains team-owned robot code. It is not intended to be published as a general-purpose library. License or usage permissions should be set by the Swartdogs team leads. If no license file exists, treat the source as team property.

## Contacts

Team leads / mentors (add names and contact info here):
- Team Lead: (add name)
- Lead Mentor: (add name)

---

If you want, I can:
- Tailor the README to the exact repository layout (I can inspect the repo and update paths).
- Add badges (build status, deploy) if you have CI set up.
- Add a short developer guide for common tasks (calibrations, setting persistent configs).
