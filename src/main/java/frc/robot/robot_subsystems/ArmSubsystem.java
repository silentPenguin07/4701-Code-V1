package frc.robot.robot_subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.ArmConstants;
import frc.robot.RobotConstants.ArmConstants.*;
import frc.robot.commands.ArmRotateCommand;


public class ArmSubsystem extends SubsystemBase
{
    
    // instance data
    
    private final Constraints m_rotationConstraints = new Constraints(RotationConstraints.MAX_ROTATION_VELOCITY_RPS, RotationConstraints.MAX_ROTATION_ACCELERATION_RPSPS);
    private final PIDController m_rotationPIDController = new PIDController(RotationGains.kP, RotationGains.kI, RotationGains.kD);
    private final RevThroughBoreEncoder m_angle_encoder = new RevThroughBoreEncoder(1);
    private ArmFeedforward m_rotationFeedforward = new ArmFeedforward(ArmConstants.ARM_LENGTH, RotationGains.kG, RotationGains.kV);
    private boolean isOpenLoopRotation = true;

    private double angleSetPoint;

    private Spark m_right;
    private Spark m_left;

    public ArmSubsystem()
    {
        m_right = new Spark(0);
        m_left = new Spark(1);

        m_rotationPIDController.enableContinuousInput(0, 2 * Math.PI);
        m_rotationPIDController.setTolerance(RotationGains.TOLERANCE.getRadians());

        isOpenLoopRotation = false;

        m_angle_encoder.setInverted(true);
        //m_angle_encoder.setOffset(ArmConstants.ARM_OFFSET_DEGREES);

        SmartDashboard.putData("Arm Rotation PID Controller", m_rotationPIDController);
    }

    public PIDController getRotationPidController()
    {
        return m_rotationPIDController;
    }

    public void armForward()
    {
        
        m_right.set(0.5);
        m_left.set(-0.5 * ArmConstants.ARM_RATIO); 
    }

    public void armBackward()
    {
        m_right.set(0.5 );
        m_left.set(-0.5 * ArmConstants.ARM_RATIO);
    }

    public void armBrake()
    {
        m_right.set(0);
        m_left.set(0);
    }

    public double getAngleSetPoint()
    {
        return angleSetPoint;
    }

    public void setAngleSetpoint(double setpoint)
    {
        angleSetPoint = setpoint;
        System.out.println("THE ANGLE WAS SET TO: " + angleSetPoint);
    }

    public boolean atSetpoint()
    {
        double currentAngle = getArmAngleDegrees();
        double target = getAngleSetPoint();
        return (currentAngle <= (target + 5) 
            && currentAngle >= (target - 5));
    }

    /*
    public void rotateClosedLoop(double velocity)
    {
        if (m_angle_encoder.isConnected())
        {
            isOpenLoopRotation = false;
            SmartDashboard.putNumber("OUTPUT", velocity);
            double feedForward = m_rotationFeedforward.calculate(getArmAngleRadians(), velocity);
            
            SmartDashboard.putNumber("FeedForward", feedForward);
            //SmartDashboard.putNumber("Voltage", RobotContainer.voltageToPercentOutput(feedForward));
            //m_rotationMotorController.set(RobotContainer.voltageToPercentOutput(feedForward));
            m_rotationMotorController.set(0.4);
        }

        else
        {
            m_rotationMotorController.set(0);
        }
    }
    */

    public double getArmAngleDegrees()
    {
        return m_angle_encoder.getAngle();
    }
    /* 
    public void setSetpoint(double Setpoint)
    {
        m_rotationPIDController.setSetpoint(Setpoint);
    }
    */

    public void periodic()
    {
        SmartDashboard.putNumber("Arm Angle", (m_angle_encoder.getAngle()));
        SmartDashboard.putNumber("Setpoint", getAngleSetPoint());
        SmartDashboard.putNumber("Error", Units.radiansToDegrees(getAngleSetPoint() - getArmAngleDegrees()));
        SmartDashboard.putBoolean("isOpenLoopRotation", isOpenLoopRotation);

    }

    public boolean isArmEncoderConnected()
    {
        return m_angle_encoder.isConnected();
    }
    
    public class RevThroughBoreEncoder
    {
        private DutyCycleEncoder m_dutyCycleEncoder;
        private double m_Offset = 0;
        private boolean m_Inverted;

        public RevThroughBoreEncoder(int dioChannel)
        {
            m_dutyCycleEncoder = new DutyCycleEncoder(dioChannel);
            m_dutyCycleEncoder.setDutyCycleRange(1.0/1024.0, 1023.0/1024.0);
            //m_dutyCycleEncoder.setDistancePerRotation(360);
            //setOffset(RobotConstants.ArmConstants.ARM_OFFSET_DEGREES);
            //m_dutyCycleEncoder.setPositionOffset((RobotConstants.ArmConstants.ARM_OFFSET_DEGREES) / 360);
        }

        /**
        public double getAngleOffset()
        {
            return RobotConstants.ArmConstants.ARM_OFFSET_DEGREES.getDegrees();
        }
        */

        public double getOffset()
        {
            return m_Offset;
        }

        public void setOffset(double m_Offset)
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

        public double getAngle()
        {
            double angle = m_dutyCycleEncoder.getAbsolutePosition() % 360;
            System.out.println(angle * 360);
            return (angle * 360);
            /* 
            angle -= m_Offset.getDegrees();

            if (m_Inverted)
            {
                angle = 360-angle;
            }

            if (angle < 0)
            {
                angle += 360;
            }
            */
            
        }
    }
    

}