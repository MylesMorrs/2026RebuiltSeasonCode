package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSubsystem {
    private static SparkMax FuelIntakeMotor1;
    private static SparkMax IntakePivotMotor1;

    public IntakeSubsystem()
    {
        FuelIntakeMotor1 = new SparkMax( IntakeConstants.FuelIntakeCanId, MotorType.kBrushless);
        IntakePivotMotor1 = new SparkMax( IntakeConstants.IntakePivotCanId, MotorType.kBrushless); 
    }


    public static void SetIntakeSpeed(Double speed)
    {
        double clamped = Math.max(-0.4, Math.min(0.4, speed));
        FuelIntakeMotor1.set(clamped);
    }

    public static void SetIntakeSpeedOverride(Double speed)
    {
        double clamped = Math.max(-1.0, Math.min(1.0, speed));
        FuelIntakeMotor1.set(clamped);
    }

     public static void SetPivotSpeed(Double speed)
    {
        IntakePivotMotor1.set(-speed);
    }



}
