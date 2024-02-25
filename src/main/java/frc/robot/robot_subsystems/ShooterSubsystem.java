package frc.robot.robot_subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    private CANSparkMax shooterMotorController;
    private Spark intakeMotorController;

    public ShooterSubsystem()
    {
        CANSparkMax clock = new CANSparkMax(0, MotorType.kBrushless);
        CANSparkMax anticlock = new CANSparkMax(1, MotorType.kBrushless);

        clock.follow(shooterMotorController);
        anticlock.setInverted(true); // invertts one motor
        anticlock.follow(shooterMotorController);

        intakeMotorController = new Spark(2);
    }

    public void shoot(int onOff)
    {
        shooterMotorController.set(0.4 * onOff);
    }


    public void intake(int posNeg)
    {
        intakeMotorController.set(0.3 * posNeg);
    }

}
