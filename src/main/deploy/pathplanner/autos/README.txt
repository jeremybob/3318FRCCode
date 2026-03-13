Place your .auto files here (created in PathPlanner desktop app).
Required auto names referenced in RobotContainer.java:
  - BlueDepot.auto
  - BlueOutpost.auto

Named commands available for events on paths:
  - HomeIntake   : Home the intake at start of auto
  - IntakeFuel   : Deploy intake, spin rollers with stall detection
  - IntakeGamePiece: Backward-compatible alias for IntakeFuel
  - AutoShoot    : Align to HUB via vision + shoot (6s timeout)

Chooser options also include:
  - Do Nothing
  - Calibrate CANcoders

Add more autos by updating RobotAutoCatalog.competitionPathPlannerAutos().
