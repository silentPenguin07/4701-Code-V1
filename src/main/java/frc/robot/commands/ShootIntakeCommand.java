package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.ShooterIntakeSubsystem;

public class ShootIntakeCommand extends Command {
    
    private final ShooterIntakeSubsystem shooterIntakeSubsystem;
    private boolean cancel;

    public ShootIntakeCommand(ShooterIntakeSubsystem shootIntake, boolean cancel)
    {
        shooterIntakeSubsystem = shootIntake;
        this.cancel = cancel;
    }

    public void initialize()
    {
        System.out.println("Shooter and/or intake running");
        shooterIntakeSubsystem.brake();
    }

    public void execute()
    {
        shooterIntakeSubsystem.intake();
        shooterIntakeSubsystem.shoot();
    }

    public void end(boolean interrupted)
    {
        shooterIntakeSubsystem.brake();
    }

    public boolean isFinished()
    {
        return cancel;
    }

}
