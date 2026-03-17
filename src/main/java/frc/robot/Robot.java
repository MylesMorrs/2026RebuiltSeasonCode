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
  private VisionSubsystem m_vision;
  private double m_autoShootPowerOffset = 0.0;
  private boolean m_turretSearchPositive = true;
  private final XboxController driver = new XboxController(OIConstants.kDriverControllerPort);




  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_vision = m_robotContainer.getVisionSubsystem();
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
    boolean pivot = m_manipulatorController.getAButton();
    double shooter = m_manipulatorController.getRightTriggerAxis();
    double transfer = -m_manipulatorController.getLeftTriggerAxis();
    boolean revtransfer = m_manipulatorController.getLeftBumperButton();
    //boolean climber = m_manipulatorController.getLeftBumperButton();
    double aiming = m_manipulatorController.getLeftX();
    boolean autoAim = m_manipulatorController.getXButton();
    boolean autoShoot = autoAim && m_manipulatorController.getRightBumperButton();
    boolean hasAutoAimTarget = false;
    boolean autoShootAligned = false;
    double yawErrorDeg = Double.NaN;
    double autoShootPower = 0.0;
    double autoTransferPower = 0.0;
    int pov = m_manipulatorController.getPOV();
    boolean fixedShoot50 = m_manipulatorController.getYButton();
    boolean fixedShoot60 = m_manipulatorController.getBButton();
    

    if (autoAim) {
      var yawOpt = m_vision.getBestTargetYawDegrees();
      var distOpt = m_vision.getBestTargetDistanceMeters();
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

      if (distOpt.isPresent()) {
        autoShootPower = computeAutoShootPower(distOpt.getAsDouble());
      }
    }

    if (autoShoot) {
      if (pov == 0) {
        m_autoShootPowerOffset += ShootingConstants.kAutoShootPowerOffsetStepPerLoop;
      } else if (pov == 180) {
        m_autoShootPowerOffset -= ShootingConstants.kAutoShootPowerOffsetStepPerLoop;
      }
      m_autoShootPowerOffset = MathUtil.clamp(
          m_autoShootPowerOffset,
          -ShootingConstants.kAutoShootPowerOffsetMax,
          ShootingConstants.kAutoShootPowerOffsetMax);
      autoShootAligned = hasAutoAimTarget
          && Math.abs(yawErrorDeg) <= ShootingConstants.TurretShootMaxYawErrorDeg;
      shooter = autoShootAligned
          ? MathUtil.clamp(autoShootPower + m_autoShootPowerOffset, 0.0, 1.0)
          : 0.0;
      transfer = autoShootAligned ? ShootingConstants.kAutoTransferSpeed : 0.0;
  }

    if (!autoShoot) {
      if (fixedShoot50) {
        shooter = 0.5;
      } else if (fixedShoot60) {
        shooter = 0.6;
      }
    }

    boolean intakeFullOverride = fixedShoot50 && fixedShoot60;
    if (intakeFullOverride) {
      intake = 1.0;
    }

    if (intakeFullOverride) {
      IntakeSubsystem.SetIntakeSpeedOverride(intake);
    } else {
      IntakeSubsystem.SetIntakeSpeed(intake);
    }
    if (pivot){
    IntakeSubsystem.SetPivotSpeed(0.8);}
    else{
      IntakeSubsystem.SetPivotSpeed(0.0);
    }
    ShootingSubsystem.SetShootingSpeed(shooter);
    if (!revtransfer){
      ShootingSubsystem.SetTransferSpeed(transfer);
    } else if (revtransfer){
      ShootingSubsystem.SetTransferSpeed(0.8);
    }
    
    //ClimberSubsystem.SetClimberSpeed(climber);
    ShootingSubsystem.SetAimingSpeed(aiming);
    SmartDashboard.putBoolean("Turret/AutoAimEnabled", autoAim);
    //SmartDashboard.putBoolean("Turret/AutoShootEnabled", autoShoot);
    SmartDashboard.putBoolean("Turret/HasAutoAimTarget", hasAutoAimTarget);
    SmartDashboard.putBoolean("Turret/AutoShootAligned", autoShootAligned);
    SmartDashboard.putNumber("Turret/AimCommand", aiming);
    SmartDashboard.putNumber("Turret/TargetYawOffsetDeg", ShootingConstants.TurretAimYawOffsetDeg);
    SmartDashboard.putNumber("Turret/ShooterCommand", shooter);
    SmartDashboard.putNumber("Turret/AutoShootPowerOffset", m_autoShootPowerOffset);
    SmartDashboard.putNumber("Joystick", driver.getLeftY());


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
  }}
 
