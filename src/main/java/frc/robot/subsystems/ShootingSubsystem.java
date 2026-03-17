package frc.robot.subsystems;
import frc.robot.Constants.ShootingConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.MathUtil;

public class ShootingSubsystem {
    private static SparkFlex ShootingMotor; 
    private static SparkMax TransferMotor; 
    private static SparkMax AimingMotor; 
    private static DutyCycleEncoder TurretEncoder;
    private static double turretZeroOffsetDeg = 0.0;
    private static double turretContinuousDeg = 0.0;
    private static double lastTurretRelDeg = 0.0;

    public ShootingSubsystem()
    {
        ShootingMotor = new SparkFlex( ShootingConstants.ShooterMotorCanId, MotorType.kBrushless);
        TransferMotor = new SparkMax( ShootingConstants.TransferMotorCanId, MotorType.kBrushless);
        AimingMotor = new SparkMax( ShootingConstants.AimingMotorCanId, MotorType.kBrushed);  
        TurretEncoder = new DutyCycleEncoder(ShootingConstants.kTurretEncoderDioChannel);
        turretZeroOffsetDeg = ShootingConstants.kTurretEncoderOffsetDeg;
        lastTurretRelDeg = getTurretRelativeDegWrapped();
        turretContinuousDeg = 0.0;
    }


    public static void SetShootingSpeed(Double speed)
    {
        ShootingMotor.set(-speed);
    }

     public static void SetTransferSpeed(Double speed)
    {
        TransferMotor.set(speed);
    }

    public static void SetAimingSpeed(Double speed)
    {
        AimingMotor.set(speed);
    }

    public static double GetTurretPositionDegrees()
    {
        return getTurretRelativeDegWrapped();
    }

    public static double GetTurretContinuousDegrees()
    {
        updateTurretContinuous();
        return turretContinuousDeg;
    }

    public static void ZeroTurretEncoder()
    {
        turretZeroOffsetDeg = getTurretRawDegrees();
        lastTurretRelDeg = getTurretRelativeDegWrapped();
        turretContinuousDeg = 0.0;
    }

    private static double getTurretRelativeDegWrapped()
    {
        return MathUtil.inputModulus(getTurretRawDegrees() - turretZeroOffsetDeg, 0.0, 360.0);
    }

    private static void updateTurretContinuous()
    {
        double currentRel = getTurretRelativeDegWrapped();
        double delta = currentRel - lastTurretRelDeg;
        if (delta > 180.0) {
            delta -= 360.0;
        } else if (delta < -180.0) {
            delta += 360.0;
        }
        turretContinuousDeg += delta;
        lastTurretRelDeg = currentRel;
    }

    private static double getTurretRawDegrees()
    {
        double encoderRotations = TurretEncoder.get();
        double turretRotations = encoderRotations / ShootingConstants.kTurretEncoderGearRatio;
        return turretRotations * 360.0;
    }


}
