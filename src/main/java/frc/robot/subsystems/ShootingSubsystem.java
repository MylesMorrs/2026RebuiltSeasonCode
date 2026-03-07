package frc.robot.subsystems;
import frc.robot.Constants.ShootingConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShootingSubsystem {
    private static SparkMax ShootingMotor; 
    private static SparkMax TransferMotor; 

    public ShootingSubsystem()
    {
        ShootingMotor = new SparkMax( ShootingConstants.ShooterMotorCanId, MotorType.kBrushless);
        TransferMotor = new SparkMax( ShootingConstants.TransferMotorCanId, MotorType.kBrushless);  
    }


    public static void SetShootingSpeed(Double speed)
    {
        ShootingMotor.set(speed);
    }

     public static void SetTransferSpeed(Double speed)
    {
        TransferMotor.set(speed);
    }



}