package frc.robot.subsystems;
import frc.robot.Constants.ShootingConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShootingSubsystem {
    private static SparkMax ShootingMotor; 
    private static SparkMax TransferMotor; 
    private static SparkMax AimingMotor; 

    public ShootingSubsystem()
    {
        ShootingMotor = new SparkMax( ShootingConstants.ShooterMotorCanId, MotorType.kBrushless);
        TransferMotor = new SparkMax( ShootingConstants.TransferMotorCanId, MotorType.kBrushless);
        AimingMotor = new SparkMax( ShootingConstants.AimingMotorCanId, MotorType.kBrushless);  
    }


    public static void SetShootingSpeed(Double speed)
    {
        ShootingMotor.set(speed);
    }

     public static void SetTransferSpeed(Double speed)
    {
        TransferMotor.set(speed);
    }

     public static void SetAimingSpeed(Double speed)
    {
        AimingMotor.set(speed);
    }



}