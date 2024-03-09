package frc.robot.robot_subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIntakeSubsystem extends SubsystemBase {
    
    private CANSparkMax clockShooter;
    private CANSparkMax antiClockShooter;
    private Spark intakeMotorController;

    public ShooterIntakeSubsystem()
    {
        clockShooter = new CANSparkMax(10, MotorType.kBrushless);
        antiClockShooter = new CANSparkMax(11, MotorType.kBrushless);

        intakeMotorController = new Spark(2);
    }
  
    public void intake(int intakePosNeg)
    {
        intakeMotorController.set(0.6 * intakePosNeg);
    }

    public void shoot(int shootPosNeg)
    {
        clockShooter.set(1 * shootPosNeg);
        antiClockShooter.set(-1 * shootPosNeg);
    }

    public void brake()
    {
        intakeMotorController.set(0);
        clockShooter.set(0);
        antiClockShooter.set(0);
    }

}
