# CLAUDE.md

Project guidance for Claude Code.

## Subsystem docs
- **Turret** — design notes, homing/drift-correction, PID/Motion Magic tuning, shoot-on-the-move,
  feed gating, and hard-won gotchas: [`src/main/java/frc/robot/subsystems/turret/README.md`](src/main/java/frc/robot/subsystems/turret/README.md).
  Read it before changing turret gains, the homing sensor logic, or the shoot-on-the-move / feed path.
- **BLine autos** — second path-following engine alongside PathPlanner; how to author autos as JSON
  routines (competition-editable, no recompile), the routine schema, and the trench-safety ordering rule:
  [`src/main/java/frc/robot/auto/README.md`](src/main/java/frc/robot/auto/README.md).
  Read it before changing BLine follower gains, routine loading, or the event-marker registry.
