package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.ShooterSubsystem;
import frc.robot.robot_subsystems.ControllerInput;

public class ShootCommand extends Command {
    
    private final ShooterSubsystem shooterIntake;

    public ShootCommand(ShooterSubsystem s)
    {
        shooterIntake = s;
    }

    public void initialize()
    {

    }

    public void execute()
    {
        /*
         * Hold down RB to engage shoot and positive intake.
         * Then, use LB (hold) to spin intake opposite for a period
         */
        if (ControllerInput.getRightBumber())
        {
            shooterIntake.intake(1);
        }
        
        if (ControllerInput.getLeftBumper())
        {
            shooterIntake.intake(-1);
        }

        if (ControllerInput.getRightTrigger())
        {
            shooterIntake.shoot(0);
        }
    }
}
