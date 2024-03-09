package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.ShooterIntakeSubsystem;


public class IntakeCommand extends Command {
    
    private final ShooterIntakeSubsystem shooterIntakeSubsystem;
    private boolean reverse; 

    public IntakeCommand(ShooterIntakeSubsystem shootIntake, boolean reverse)
    {
        shooterIntakeSubsystem = shootIntake;
        this.reverse = reverse;
    }

    public void initialize()
    {
        System.out.println("Shooter and/or intake running");
        shooterIntakeSubsystem.brake();
    }

    public void execute()
    {
        
        if (reverse)
        {
            shooterIntakeSubsystem.intake(1);
        }
        else
        {
            shooterIntakeSubsystem.intake(-1);
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
