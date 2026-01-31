package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSubsystem {
    private static SparkMax FuelIntakeMotor;
    private static SparkMax IntakePivotMotor1;
    private static SparkMax IntakePivotMotor2; 

    public IntakeSubsystem()
    {
        FuelIntakeMotor = new SparkMax( IntakeConstants.FuelIntakeCanId, MotorType.kBrushless);
        IntakePivotMotor1 = new SparkMax( IntakeConstants.IntakePivot1CanId, MotorType.kBrushless); 
        IntakePivotMotor2 = new SparkMax( IntakeConstants.IntakePivot2CanId, MotorType.kBrushless); 


    }


    public static void SetIntakeSpeed(Double speed)
    {
        FuelIntakeMotor.set(speed);
    }

     public static void SetPivotSpeed(Double speed)
    {
        IntakePivotMotor1.set(speed);
        IntakePivotMotor2.set(speed);
    }



}
