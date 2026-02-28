Place your .auto files here (created in PathPlanner desktop app).
Required auto names referenced in RobotContainer.java:
  - EightFuelClimbAuto.auto
  - TaxiOnly.auto

Named commands available for events on paths:
  - HomeIntake   : Home the intake at start of auto
  - IntakeFuel   : Deploy intake, spin rollers with stall detection
  - AutoShoot    : Align to HUB via vision + shoot (6s timeout)
  - Level1Climb  : Automatic Level 1 climb (15 pts in auto)

You can add more and register them in RobotContainer.configureAutoChooser().
