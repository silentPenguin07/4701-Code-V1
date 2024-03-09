package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.ShooterIntakeSubsystem;


public class ShootCommand extends Command {
    
    private final ShooterIntakeSubsystem shooterIntakeSubsystem;
    private boolean reverse; 

    public ShootCommand(ShooterIntakeSubsystem shootIntake, boolean cancel)
    {
        shooterIntakeSubsystem = shootIntake;
        this.reverse = cancel;
    }

    public void initialize()
    {
        System.out.println("Shooter and/or intake running");
        shooterIntakeSubsystem.brake();
    }

    public void execute()
    {
        if (!reverse)
        {
            shooterIntakeSubsystem.shoot(1);
        }
        else{
            shooterIntakeSubsystem.shoot(-1);
        }
    }

    public void end(boolean interrupted)
    {
        shooterIntakeSubsystem.brake();
    }

    public boolean isFinished()
    {
        return false;
    }

}
