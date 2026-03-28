package frc.robot.subsystems;
import frc.robot.Constants.ClimbingConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ClimberSubsystem {
    private static SparkMax ClimbingMotor1;
    private static SparkMax ClimbingMotor2; 

    public ClimberSubsystem(){
    ClimbingMotor1 = new SparkMax(ClimbingConstants.ClimbingMotor1CanId, MotorType.kBrushless);
    ClimbingMotor2 = new SparkMax(ClimbingConstants.ClimbingMotor2CanId, MotorType.kBrushless);
    }

    public static void SetClimberSpeed(Double speed){
        ClimbingMotor1.set(speed);
        ClimbingMotor2.set(-speed);

    }
}
