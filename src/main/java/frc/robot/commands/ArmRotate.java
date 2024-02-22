package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import entechlib.commands.EntechCommand;
import frc.robot.robot_subsystems.ArmSubsystem;

public class ArmRotate extends EntechCommand {
    
    private ArmSubsystem m_arm;
    private double m_angleRadians;
    private final XboxController joystick;

    // constructor
    public ArmRotate(ArmSubsystem arm, double angleRadians, XboxController joystick)
    {
        this.m_arm = arm;
        this.m_angleRadians = angleRadians;
        this.joystick = joystick;
    }

    // called when command is initially scheduled
    public void initialize()
    {
        m_arm.getRotationPidController().reset();
    }

    // called every time thje scheduler runs while the command is scheduled
    public void execute()
    {
        PIDController rotationController = m_arm.getRotationPidController();
        rotationController.setSetpoint(m_angleRadians);
        m_arm.rotateClosedLoop(rotationController.calculate(m_arm.getArmAngleRadians()));
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
