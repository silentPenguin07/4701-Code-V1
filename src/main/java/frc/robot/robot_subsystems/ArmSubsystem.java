package frc.robot.robot_subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ArmConstants;
import frc.robot.RobotConstants.ArmConstants.*;
import frc.robot.RobotContainer;


public class ArmSubsystem extends SubsystemBase
{

    public class RevThroughBoreEncoder
    {
        private DutyCycleEncoder m_dutyCycleEncoder;
        private Rotation2d m_Offset = Rotation2d.fromDegrees(0);
        private boolean m_Inverted;

        public RevThroughBoreEncoder(int dioChannel)
        {
            m_dutyCycleEncoder = new DutyCycleEncoder(dioChannel);
            m_dutyCycleEncoder.setDutyCycleRange(1.0/1024.0, 1023.0/1024.0);
            m_dutyCycleEncoder.setDistancePerRotation(360);
        }

        public Rotation2d getOffset()
        {
            return m_Offset;
        }

        public void setOffset(Rotation2d m_Offset)
        {
            this.m_Offset = m_Offset;
        }

        public boolean isInverted()
        {
            return m_Inverted;
        }

        public void setInverted(boolean inverted)
        {
            m_Inverted = inverted;
        }

        public boolean isConnected()
        {
            return (m_dutyCycleEncoder.isConnected());
        }

        public Rotation2d getAngle()
        {
            double angle = m_dutyCycleEncoder.getDistance() % 360;
            
            angle -= m_Offset.getDegrees();

            if (m_Inverted)
            {
                angle = 360-angle;
            }

            if (angle < 0)
            {
                angle += 360;
            }

            return Rotation2d.fromDegrees(angle % 360);
        }
    }


    // instiance data
    private Spark m_rotationMotorController;
    private final Constraints m_rotationConstraints = new Constraints(RotationConstraints.MAX_ROTATION_VELOCITY_RPS, RotationConstraints.MAX_ROTATION_ACCELERATION_RPSPS);
    private final PIDController m_rotationPIDController = new PIDController(RotationGains.kP, RotationGains.kI, RotationGains.kD);
    // TODO: Should be changed
    private final RevThroughBoreEncoder m_angle_encoder = new RevThroughBoreEncoder(0); // diochannel needs to be looked at
    private ArmFeedforward m_rotationFeedforward = new ArmFeedforward(ArmConstants.ARM_LENGTH, RotationGains.kG, RotationGains.kV); // TODO: Arm length
    private double angleSetPointRadians;
    private boolean isOpenLoopRotation = true;

    public ArmSubsystem()
    {
        /* not really needed
        leftArm = new Spark(5);
        rightArm = new Spark(6);
        rightArm.setInverted(true);
        */
        m_rotationMotorController.addFollower(new Spark(5));

        // inverted motor (easier to make separately)
        Spark inv = new Spark(6);
        inv.setInverted(true);
        m_rotationMotorController.addFollower(inv);

        m_angle_encoder.setInverted(true);
        m_angle_encoder.setOffset(ArmConstants.ARM_OFFSET_DEGREES);
        setAngleSetpointRadians(getArmAngleRadians());
    }

    public PIDController getRotationPidController()
    {
        return m_rotationPIDController;
    }

    public Command rotateToCommand(Rotation2d angle)
    {
        return runOnce(() -> setAngleSetpointRadians(angle.getRadians()));
    }

    public void rotate(double percent)
    {
        SmartDashboard.putNumber("RotateRotercent", percent);

        // open loop rotation stuff
    }

    public void rotateClosedLoop(double velocity)
    {
        if (m_angle_encoder.isConnected())
        {
            isOpenLoopRotation = false;
            SmartDashboard.putNumber("OUTPUT", velocity);
            double feedForward = m_rotationFeedforward.calculate(getArmAngleRadians(), velocity);

            // TODO: Do calculations for feed forward, output and apply to motor percent output
            //double extendedFeedForward = m_rotationFeedforwardExtended.calculate(getArmAngleRadians(), velocity);
            
            //double finalFF = MathUtil.interpolate(feedForward, ArmConstants.ARM_LENGTH);
            
            //SmartDashboard.putNumber("FeedForward", combinedFF);
            //SmartDashboard.putNumber("Voltage", RobotContainer.voltageToPercentOutput(combinedFF));
            //m_rotationMotorController.set(ControlMode.PercentOutput, RobotContainer.voltageToPercentOutput(combinedFF));
        }

        else
        {
            m_rotationMotorController.set(0);
            // m_rotationMotorController.set(RobotContainer.voltageToPercentOutput());
        }
    }

    public double getArmAngleRadians()
    {
        return m_angle_encoder.getAngle().getRadians();
    }

    public double getAngleSetpointRadians()
    {
        return angleSetPointRadians;
    }

    public void setAngleSetpointRadians(double angleSetpoint)
    {
        this.angleSetPointRadians = angleSetpoint;
    }

    public void hold()
    {
        // rotate closed loop (0)
        m_rotationMotorController.set(0);
    }

    public void periodic()
    {
        SmartDashboard.putNumber("Arm Angle", (m_angle_encoder.getAngle().getDegrees()));
        SmartDashboard.putNumber("Setpoint", Units.radiansToDegrees((getAngleSetpointRadians())));
        SmartDashboard.putNumber("Measurement", Units.radiansToDegrees(getArmAngleRadians()));
        SmartDashboard.putNumber("Error", Units.radiansToDegrees(getAngleSetpointRadians() - getArmAngleRadians()));
        SmartDashboard.putBoolean("isOpenLoopRotation", isOpenLoopRotation);

    }

    public boolean isArmEncoderConnected()
    {
        return m_angle_encoder.isConnected();
    }
    

}