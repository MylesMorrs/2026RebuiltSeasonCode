package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.VisionSubsystem;
//import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  private final DriveSubsystem drive = new DriveSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();
  private final XboxController driver = new XboxController(OIConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;
  private boolean fieldRelative = true;
  private final Command autoShootCommand;
  private boolean turretSearchPositive = true;

  public RobotContainer() {
    autoShootCommand = Commands.run(this::runAutoShootControl)
        .finallyDo(interrupted -> stopAutoShootOutputs());

    // Register NamedCommands here if needed (before building chooser)
    NamedCommands.registerCommand(
        "Shoot",
        Commands.run(
          () -> ShootingSubsystem.SetShootingSpeed(0.5))
          .withTimeout(ShootingConstants.kAutoShootSeconds)); 
    NamedCommands.registerCommand(
        "stop Shoot",
        Commands.run(
          () -> ShootingSubsystem.SetShootingSpeed(0.0))
          .withTimeout(ShootingConstants.kAutoShootSeconds));      
        
    NamedCommands.registerCommand(
        "drop intake",
        Commands.startEnd(
            () -> IntakeSubsystem.SetPivotSpeed(IntakeConstants.kAutoIntakeDropSpeed),
            () -> IntakeSubsystem.SetPivotSpeed(0.0))
            .withTimeout(IntakeConstants.kAutoIntakeDropSeconds));
    NamedCommands.registerCommand(
        "start transfer",
        Commands.run(
            () -> ShootingSubsystem.SetTransferSpeed(-0.8))
            .withTimeout(ShootingConstants.kAutoShootSeconds));
    NamedCommands.registerCommand(
        "stop Transfer",
        Commands.run(
            () -> ShootingSubsystem.SetTransferSpeed(0.0))
            .withTimeout(ShootingConstants.kAutoShootSeconds));
    NamedCommands.registerCommand(
        "start Intake",
        Commands.run(
            () -> IntakeSubsystem.SetIntakeSpeed(IntakeConstants.kAutoIntakeSpeed)
            )
            .withTimeout(IntakeConstants.kAutoIntakeSeconds));
    NamedCommands.registerCommand(
        "Start Auto Shoot",
        Commands.runOnce(this::startAutoShoot));
    NamedCommands.registerCommand(
        "Start Shoot",
        Commands.runOnce(this::startAutoShoot));

    // Build auto chooser with a DEFAULT auto name that matches a .auto file
    autoChooser = AutoBuilder.buildAutoChooser("new auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureButtonBindings();

    drive.setDefaultCommand(new RunCommand(() -> {
      double db = OIConstants.kDriveDeadband;
      double slow = driver.getRightBumper() ? OIConstants.kDriveSlowMultiplier : 1.0;
      double x   = -MathUtil.clamp(MathUtil.applyDeadband(driver.getLeftY(),  db)  * OIConstants.kDriveInputGain, -2.0, 2.0);
      double y   = -MathUtil.clamp(MathUtil.applyDeadband(driver.getLeftX(),  db) * OIConstants.kDriveInputGain, -2.0, 2.0);
      double rot = -MathUtil.clamp(MathUtil.applyDeadband(driver.getRightX(), db) * OIConstants.kTurnInputGain, -2.0, 2.0);
      drive.drive(x, y, rot, fieldRelative);
    }, drive));

    SmartDashboard.putData("Field", drive.getField());
  }

  public VisionSubsystem getVisionSubsystem() {
    return vision;
  }

  private void configureButtonBindings() {
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(drive::setX, drive));

    new JoystickButton(driver, XboxController.Button.kY.value)
        .onTrue(new RunCommand(() -> fieldRelative = !fieldRelative).ignoringDisable(true));

    new JoystickButton(driver, XboxController.Button.kStart.value)
        .onTrue(new RunCommand(drive::zeroHeading, drive));
  }

  public Command getAutonomousCommand() {
    Command chosen = autoChooser.getSelected();
    if (chosen != null) return chosen;
    // Fallback to the default name in case chooser didn't populate
    return AutoBuilder.buildAuto("L3 CoralScoring");
  }

  private void runAutoShootControl() {
    double aiming = 0.0;
    double shooter = 0.0;
    double transfer = 0.0;

    var yawOpt = vision.getBestTargetYawDegrees();
    var distOpt = vision.getBestTargetDistanceMeters();

    if (yawOpt.isPresent() && distOpt.isPresent()) {
      double yawErrorDeg = yawOpt.getAsDouble() - ShootingConstants.TurretAimYawOffsetDeg;
      if (Math.abs(yawErrorDeg) >= ShootingConstants.TurretAimDeadbandDeg) {
        aiming = MathUtil.clamp(
            yawErrorDeg * ShootingConstants.TurretAimKp,
            -ShootingConstants.TurretAimMaxSpeed,
            ShootingConstants.TurretAimMaxSpeed);
      }

      shooter = computeAutoShootPower(distOpt.getAsDouble());
      transfer = ShootingConstants.kAutoTransferSpeed;
    } else {
      aiming = 0.0;
    }

    ShootingSubsystem.SetAimingSpeed(aiming);
    ShootingSubsystem.SetShootingSpeed(shooter);
    ShootingSubsystem.SetTransferSpeed(transfer);
  }

  private void stopAutoShootOutputs() {
    ShootingSubsystem.SetAimingSpeed(0.0);
    ShootingSubsystem.SetShootingSpeed(0.0);
    ShootingSubsystem.SetTransferSpeed(0.0);
  }

  private void startAutoShoot() {
    if (!CommandScheduler.getInstance().isScheduled(autoShootCommand)) {
      autoShootCommand.schedule();
    }
  }

  private void stopAutoShoot() {
    if (CommandScheduler.getInstance().isScheduled(autoShootCommand)) {
      autoShootCommand.cancel();
    }
    stopAutoShootOutputs();
  }

  private double computeAutoShootPower(double distanceMeters) {
    double minD = ShootingConstants.kAutoShootMinDistanceMeters;
    double maxD = ShootingConstants.kAutoShootMaxDistanceMeters;
    double minP = ShootingConstants.kAutoShootMinPower;
    double maxP = ShootingConstants.kAutoShootMaxPower;

    if (maxD <= minD) {
      return maxP;
    }

    double t = (distanceMeters - minD) / (maxD - minD);
    t = MathUtil.clamp(t, 0.0, 1.0);
    return minP + (maxP - minP) * t;
  }
}
