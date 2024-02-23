package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import entechlib.commands.EntechCommand;
import frc.robot.robot_subsystems.ArmSubsystem;
import frc.robot.RobotConstants.ArmConstants.RotationSetpoints;;

public class ArmRotateCommand extends EntechCommand {
    
    private ArmSubsystem m_arm;
    //private double m_angleRadians;
    private final XboxController joystick;

    // constructor
    public ArmRotateCommand(ArmSubsystem arm, /*double angleRadians,*/ XboxController joystick)
    {
        this.m_arm = arm;
        //this.m_angleRadians = angleRadians;
        this.joystick = joystick;
    }

    // called when command is initially scheduled
    public void initialize()
    {
        m_arm.getRotationPidController().reset();
        PIDController rotationController = m_arm.getRotationPidController();

        // set arm to 0 position
        rotationController.setSetpoint(Units.degreesToRadians(0));
    }

    // called every time thje scheduler runs while the command is scheduled
    public void execute()
    {
        PIDController rotationController = m_arm.getRotationPidController();
        

        // move to low position
        if (joystick.getAButtonPressed())
        {
            rotationController.setSetpoint(RotationSetpoints.LOW_RADIANS);
            m_arm.rotateClosedLoop(rotationController.calculate(m_arm.getArmAngleRadians()));
        }
        
        // move to mid position
        else if (joystick.getBButtonPressed())
        {
            rotationController.setSetpoint(RotationSetpoints.LOW_RADIANS);
            m_arm.rotateClosedLoop(rotationController.calculate(m_arm.getArmAngleRadians()));
        }

        // move to high position
        else if (joystick.getXButtonPressed())
        {
            rotationController.setSetpoint(RotationSetpoints.LOW_RADIANS);
            m_arm.rotateClosedLoop(rotationController.calculate(m_arm.getArmAngleRadians()));
        }
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
