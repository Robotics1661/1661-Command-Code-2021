<div align="center">

<img src="https://capsule-render.vercel.app/api?type=waving&height=140&color=0:aa0000,50:d40000,100:ff1a1a&section=header" alt="" width="100%" />

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://team1661.com/assets/images/1661-logo-white.png">
  <img src="https://team1661.com/assets/images/1661-logo-black.png" alt="Team 1661 logo" width="120">
</picture>

# 🤖 1661-Command-Code-2021

**Command-based robot code for the 2021-22 season**

[![FRC: Team 1661](https://img.shields.io/badge/FRC-Team%201661-d40000?style=for-the-badge&logo=firstinspires&logoColor=white)](https://team1661.com)
![Java: 11](https://img.shields.io/badge/Java-11-ED8B00?style=for-the-badge&logo=openjdk&logoColor=white)
[![WPILib: 2022](https://img.shields.io/badge/WPILib-2022-003366?style=for-the-badge)](https://docs.wpilib.org/)
[![License](https://img.shields.io/github/license/Robotics1661/1661-Command-Code-2021?style=for-the-badge&color=d40000)](LICENSE)

</div>

---

Command-based Java robot code from [Griffitrons 1661](https://team1661.com), the FIRST Robotics Competition team at The Buckley School for the 2021–22 season (the repo name predates the season change).

## ✨ Features

- Subsystems for the drivetrain, intake, shooter, vertical and horizontal agitators, ultrasonic sensor and vision
- Commands for intaking and ejecting balls, high and low shots (including `SmartShootHigh` and
  `InstantHighShot`), target finding and teleop driving
- Autonomous routine plus [PathWeaver](https://docs.wpilib.org/en/stable/docs/software/pathplanning/pathweaver/index.html)
  paths in `PathWeaver/`

The `TimedRobot` version of the same robot lives in
[1661-Code-2022](https://github.com/Robotics1661/1661-Code-2022).

## 🛠️ Building and running

You'll need [WPILib 2022](https://docs.wpilib.org/) (it bundles the JDK and Gradle). Then, from the repo root:

```bash
./gradlew build          # compile and run checks
./gradlew simulateJava   # run in the WPILib simulator
./gradlew deploy         # deploy to a roboRIO (must be connected to the robot)
```

On Windows use `gradlew.bat` instead of `./gradlew`. The team number (1661) lives in
`.wpilib/wpilib_preferences.json`.

## 📜 License

Team-written code is released under the [MIT License](LICENSE). Files generated from the WPILib
project template keep their original headers, and WPILib itself is covered by the BSD-style license in
[WPILib-License.md](WPILib-License.md). Vendor libraries in `vendordeps/` are covered by their own licenses.

---

<div align="center">

<sub>Built by students of <a href="https://team1661.com">Griffitrons 1661</a> at The Buckley School 🛠️</sub>

<img src="https://capsule-render.vercel.app/api?type=waving&height=90&color=0:aa0000,50:d40000,100:ff1a1a&section=footer" alt="" width="100%" />

</div>
