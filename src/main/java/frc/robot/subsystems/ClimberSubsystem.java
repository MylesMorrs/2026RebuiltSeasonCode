package frc.robot.subsystems;
import frc.robot.Constants.ClimbingConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ClimberSubsystem {
    private static SparkMax ClimbingMotor;

    public ClimberSubsystem(){
    ClimbingMotor = new SparkMax(ClimbingConstants.ClimbingMotorCanId, MotorType.kBrushless);
    }

    public static void SetClimberSpeed(Double speed){
        ClimbingMotor.set(speed); 
    }
}
