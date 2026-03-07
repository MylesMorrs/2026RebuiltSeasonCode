package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;


public class Robot extends TimedRobot {

  private Joystick m_Joystick; 

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_Joystick = new Joystick(OIConstants.kManipulatorControllerPort); 
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
    IntakeSubsystem.SetIntakeSpeed(m_Joystick.getRawAxis(Axis.kRightY.value));
    IntakeSubsystem.SetPivotSpeed(m_Joystick.getRawAxis(Axis.kRightX.value));
     

    
    ShootingSubsystem.SetShootingSpeed(m_Joystick.getRawAxis(Axis.kRightTrigger.value));
    if (m_Joystick.getRawAxis(Axis.kRightTrigger.value) > 10){
    ShootingSubsystem.SetTransferSpeed(0.5);
    }

    ClimberSubsystem.SetClimberSpeed(m_Joystick.getRawAxis(Axis.kLeftY.value))
}
  }

 
