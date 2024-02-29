package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.ArmSubsystem;
import frc.robot.RobotConstants.ArmConstants.RotationSetpoints;
import frc.robot.robot_subsystems.ControllerInput;

public class ArmRotateCommand extends Command {
    
    private final ArmSubsystem m_arm;
    private final double position; // in degrees

    // constructor

    // precondition: pos is a degree value

    public ArmRotateCommand(ArmSubsystem arm, double pos)
    {
        this.m_arm = arm;
        position = pos;
        addRequirements(m_arm);
    }

    // called when command is initially scheduled
    public void initialize()
    {
        System.out.println("Setting arm to " + position);
        m_arm.armBrake();
    }

    // called every time the scheduler runs while the command is scheduled
    public void execute()
    {
        if (m_arm.getArmAngleDegrees() < m_arm.getAngleSetpointDegrees())
        {
            m_arm.armForward();
        }
        else
        {
            m_arm.armBackward();
        }
    }

    // calle once the command ends or is interrupted
    public void end(boolean interrupted)
    {
        m_arm.armBrake();
    }

    // returns true when the command should end
    public boolean isFinished()
    {
        return m_arm.atSetpoint();
    }

}
