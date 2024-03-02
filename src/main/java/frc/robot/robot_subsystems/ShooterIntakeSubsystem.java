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
        clockShooter = new CANSparkMax(0, MotorType.kBrushless);
        antiClockShooter = new CANSparkMax(1, MotorType.kBrushless);
        antiClockShooter.setInverted(true); // invertts one motor
        antiClockShooter.follow(clockShooter);

        intakeMotorController = new Spark(2);
    }

    public void intake()
    {
        intakeMotorController.set(0.6);
    }

    public void shoot()
    {
        clockShooter.set(0.6);
    }

    public void brake()
    {
        intakeMotorController.set(0);
        clockShooter.set(0);
    }

}
