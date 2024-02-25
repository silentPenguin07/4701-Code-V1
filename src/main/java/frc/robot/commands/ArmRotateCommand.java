package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.ArmSubsystem;
import frc.robot.RobotConstants.ArmConstants.RotationSetpoints;
import frc.robot.robot_subsystems.ControllerInput;

public class ArmRotateCommand extends Command {
    
    private ArmSubsystem m_arm;
    //private double m_angleRadians;

    // constructor
    public ArmRotateCommand(ArmSubsystem arm /*XboxController joystick*/)
    {
        this.m_arm = arm;
        //this.m_angleRadians = angleRadians;
    }

    // called when command is initially scheduled
    public void initialize(String input)
    {
        m_arm.getRotationPidController().reset();
        PIDController rotationController = m_arm.getRotationPidController();

        // set arm to 0 position
        

        // move to low position
        if (input.equals("A"))
        {
            rotationController.setSetpoint(RotationSetpoints.LOW_RADIANS);
            //m_arm.rotateClosedLoop(rotationController.calculate(m_arm.getArmAngleRadians()));
        }
        
        // move to mid position
        else if (input.equals("B"))
        {
            rotationController.setSetpoint(RotationSetpoints.MID_RADIANS);
            //m_arm.rotateClosedLoop(rotationController.calculate(m_arm.getArmAngleRadians()));
        }

        // move to high position
        else if (input.equals("X"))
        {
            rotationController.setSetpoint(RotationSetpoints.HIGH_RADIANS);
            //m_arm.rotateClosedLoop(rotationController.calculate(m_arm.getArmAngleRadians()));
        }
    }

    // called every time thje scheduler runs while the command is scheduled
    public void execute(String input)
    {
        
    }

    // calle once the command ends or is interrupted
    public void end(boolean interrupted)
    {
        m_arm.hold();
    }

    // returns true when the command should end
    public boolean isFinished()
    {
        return m_arm.getRotationPidController().atSetpoint();
    }

}
