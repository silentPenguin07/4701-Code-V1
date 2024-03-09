package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.ArmSubsystem;

public class ArmBackwardCommand extends Command {
    
    private ArmSubsystem m_arm;

    public ArmBackwardCommand(ArmSubsystem arm)
    {
        m_arm = arm;
        addRequirements(m_arm);
    }

    public void initialize()
    {
        m_arm.armBrake();
    }

    public void execute()
    {
        m_arm.armBackward();
    }

    public void end(boolean interrupted)
    {
        m_arm.armBrake();
    }

    public boolean isFinished()
    {
        return false;
    }

}
