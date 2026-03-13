package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class Robot extends TimedRobot {

  private XboxController m_manipulatorController;

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShootingSubsystem m_shooter = new ShootingSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();



  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_manipulatorController = new XboxController(OIConstants.kManipulatorControllerPort);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.println("[AUTO] Selected: " + (m_autonomousCommand == null ? "null" : m_autonomousCommand.getName()));
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    double intake = m_manipulatorController.getRightY();
    double pivot = m_manipulatorController.getRightX();
    double shooter = m_manipulatorController.getRightTriggerAxis();
    double transfer = -m_manipulatorController.getLeftTriggerAxis();
    double climber = m_manipulatorController.getLeftY();
    double aiming = m_manipulatorController.getLeftX();
    boolean autoAim = m_manipulatorController.getXButton();
    boolean autoShoot = autoAim && m_manipulatorController.getRightBumperButton();
    boolean hasAutoAimTarget = false;
    boolean autoShootAligned = false;
    double yawErrorDeg = Double.NaN;

    if (autoAim) {
      var yawOpt = m_vision.getBestTargetYawDegrees();
      if (yawOpt.isPresent()) {
        hasAutoAimTarget = true;
        yawErrorDeg = yawOpt.getAsDouble() - ShootingConstants.TurretAimYawOffsetDeg;
        if (Math.abs(yawErrorDeg) < ShootingConstants.TurretAimDeadbandDeg) {
          aiming = 0.0;
        } else {
          aiming = MathUtil.clamp(
              yawErrorDeg * ShootingConstants.TurretAimKp,
              -ShootingConstants.TurretAimMaxSpeed,
              ShootingConstants.TurretAimMaxSpeed);
        }
        SmartDashboard.putNumber("Turret/YawErrorDeg", yawErrorDeg);
      } else {
        aiming = 0.0;
        SmartDashboard.putNumber("Turret/YawErrorDeg", 999.0);
      }
    }

    if (autoShoot) {
      autoShootAligned = hasAutoAimTarget
          && Math.abs(yawErrorDeg) <= ShootingConstants.TurretShootMaxYawErrorDeg;
      shooter = autoShootAligned ? ShootingConstants.TurretAutoShootPower : 0.0;
    }

    IntakeSubsystem.SetIntakeSpeed(intake);
    IntakeSubsystem.SetPivotSpeed(pivot);
    ShootingSubsystem.SetShootingSpeed(shooter);
    ShootingSubsystem.SetTransferSpeed(transfer);
    ClimberSubsystem.SetClimberSpeed(climber);
    ShootingSubsystem.SetAimingSpeed(aiming);
    SmartDashboard.putBoolean("Turret/AutoAimEnabled", autoAim);
    SmartDashboard.putBoolean("Turret/AutoShootEnabled", autoShoot);
    SmartDashboard.putBoolean("Turret/HasAutoAimTarget", hasAutoAimTarget);
    SmartDashboard.putBoolean("Turret/AutoShootAligned", autoShootAligned);
    SmartDashboard.putNumber("Turret/AimCommand", aiming);
    SmartDashboard.putNumber("Turret/TargetYawOffsetDeg", ShootingConstants.TurretAimYawOffsetDeg);
    SmartDashboard.putNumber("Turret/ShooterCommand", shooter);


}
  }

 
