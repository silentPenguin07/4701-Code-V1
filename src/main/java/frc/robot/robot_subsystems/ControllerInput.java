package frc.robot.robot_subsystems;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotConstants;

public class ControllerInput {
    
    public static final XboxController m_XboxController = 
        new XboxController(RobotConstants.Ports.CONTROLLER.ARM_JOYSTICK);

    public static boolean getA()
    {
        return m_XboxController.getAButtonPressed();
    }

    public static boolean getB()
    {
        return m_XboxController.getBButtonPressed();
    }

    public static boolean getX()
    {
        return m_XboxController.getXButtonPressed();
    }

    public static boolean getY()
    {
        return m_XboxController.getYButtonPressed();
    }

    public static boolean getRightBumber()
    {
        return m_XboxController.getRightBumperPressed();
    }

    public static boolean getLeftBumper()
    {
        return m_XboxController.getLeftBumperPressed();
    }

    public static boolean getRightTrigger()
    {
        return m_XboxController.getRightTriggerAxis() > 0;
    }
}
