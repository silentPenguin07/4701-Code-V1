package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class RobotConstants {

    public static final class DrivetrainConstants {
        // Driving Parameters - Not the maximum capable speeds, but the allowed maximum speeds

        // Change these if needed -- otherwise keep the same
        public static final double MAX_SPEED_METERS_PER_SECOND = 4.0; // 4.42; //4.8;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI; // radians per second

        public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
        public static final double MAGNITUDE_SLEW_RATE = 1.8; // 2.0; //1.8; // percent per second (1 = 100%)
        public static final double ROTATIONAL_SLEW_RATE = 2.0; // 20.0; //2.0; // percent per second (1 = 100%)

        // width of the chassis
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(27.5);

        // Distance between centers of right and left wheels on robot
        public static final double WHEEL_BASE_METERS = Units.inchesToMeters(22.25);

        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
                new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));


        // TODO If the gyro (the one located in subsystems/driveSubSystem) is still not reversed, change this to true and test again. Started in false.
        public static final boolean kGyroReversed = false;
    }

    public static final class SwerveModuleConstants {

        public static final double FREE_SPEED_RPM = 5676;

        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER_METERS = 0.1016;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 17 * 50) / (kDrivingMotorPinionTeeth * 15 * 27);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
                * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION = (WHEEL_DIAMETER_METERS
                * Math.PI) / DRIVING_MOTOR_REDUCTION; // meters, per rotation
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM = ((WHEEL_DIAMETER_METERS
                * Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second, per RPM

        public static final double TURNING_MOTOR_REDUCTION = 150.0 / 7.0; // ratio between internal relative encoder and
                                                                          // Through Bore (or Thrifty in our case)
                                                                          // absolute encoder - 150.0 / 7.0

        public static final double TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION = (2 * Math.PI)
                / TURNING_MOTOR_REDUCTION; // radians, per rotation
        public static final double TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM = (2 * Math.PI)
                / TURNING_MOTOR_REDUCTION / 60.0; // radians per second, per RPM

        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0; // radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS = (2 * Math.PI); // radians

        public static final double DRIVING_P = 0.04;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0;
        public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
        public static final double DRIVING_MIN_OUTPUT_NORMALIZED = -1;
        public static final double DRIVING_MAX_OUTPUT_NORMALIZED = 1;

        public static final double TURNING_P = 1.0; // 1.0; // 1.0 might be a bit too much - reduce a bit if needed
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 0;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT_NORMALIZED = -1;
        public static final double TURNING_MAX_OUTPUT_NORMALIZED = 1;

        public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

        public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 40; // 50; // amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20; // amps
        
    }


    public static interface Ports {
        public static class ANALOG {
            public static final int FRONT_LEFT_TURNING_ABSOLUTE_ENCODER = 1;
            public static final int REAR_LEFT_TURNING_ABSOLUTE_ENCODER = 3;
            public static final int FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER = 0;
            public static final int REAR_RIGHT_TURNING_ABSOLUTE_ENCODER = 2;
        }

        public static class CAN {
            public static final int FRONT_LEFT_DRIVING = 3;
            public static final int REAR_LEFT_DRIVING = 5;
            public static final int FRONT_RIGHT_DRIVING = 1;
            public static final int REAR_RIGHT_DRIVING = 7;

            public static final int FRONT_LEFT_TURNING = 4;
            public static final int REAR_LEFT_TURNING = 6;
            public static final int FRONT_RIGHT_TURNING = 2;
            public static final int REAR_RIGHT_TURNING = 8;
        }

        public static class CONTROLLER {
            public static final double JOYSTICK_AXIS_THRESHOLD = 0.2;
            public static final int DRIVER_JOYSTICK = 0;
            public static final int ARM_JOYSTICK = 1;
            public static final int PANEL = 2;
        }
    }

    /*
    // TODO: Might need these later
    public static class AutoConstants 
    {
        // PID constants for path planner (these control drive direction not reaching target wheel speeds)
		public static final double PathPlannerP = .5;
		public static final double PathPlannerI = 0;
		public static final double PathPlannerD = 0;
        public static final double PathPlannerTurnP = .8;
		public static final double PathPlannerTurnI = 0;
		public static final double PathPlannerTurnD = 0;
    }
    */

    /*
     * constants pertaining to the arm
     */
    public static class ArmConstants
    {
        // TODO: need to be changed
        public static final double ARM_LENGTH = 10; // TODO: MEASURE THE ARM
        public static final Rotation2d ARM_OFFSET_DEGREES = Rotation2d.fromDegrees(-153.4);
        public static final double SPOOL_DIAMETER = Units.inchesToMeters(2.0);
        public static final double DISTANCE_PER_SPOOL_REVOLUTION_METERS = SPOOL_DIAMETER * Math.PI;
        public static final double SPOOL_ROTATIONS_PER_METER = 1/DISTANCE_PER_SPOOL_REVOLUTION_METERS;
        public static final double FALCON_ROTATIONS_PER_SPOOL = 27.0;
        public static final double PULSES_PER_FALCON_ROTATION = 2048;
        public static final double PULSES_PER_METER_EXTENSION = SPOOL_ROTATIONS_PER_METER * FALCON_ROTATIONS_PER_SPOOL * PULSES_PER_FALCON_ROTATION;

        public static final class RotationGains
        {
            // TODO: need to be changed
            public static final double kGPercent = 0.075;
            public static final Rotation2d TOLERANCE= Rotation2d.fromDegrees(2.5);
            //public static final double kP = 8.0735;
            public static final double kP = 2.9;
            public static final double kI = 0;
            public static final double kD = 0.50318; //1.7543;
            public static final double kF = 0;
            public static final double kS = 0.1181;
            //public static final double kG = 0.32446;
            public static final double kG = 0.4;
            public static final double kV = 3.4;
            public static final double kA = 0.044465;

            /* TODO: determine whether this is needed
            public static final double kSExtended = 0.019459;
            public static final double kGExtended = 0.68405;
            public static final double kVExtended = 3.722;
            public static final double kAExtended = 0.088359;
            */
        }
        
        // TODO: values need to be changed
        public static final class RotationConstraints 
        {
            public static final double MAX_ROTATION_VELOCITY_RPS = 3 * Math.PI / 2;
            public static final double MAX_ROTATION_ACCELERATION_RPSPS = MAX_ROTATION_VELOCITY_RPS * 3;
        }

        public static final class RotationSetpoints {
            //in degrees initially, conv to rad
            public static final double LOW_RADIANS = Units.degreesToRadians(-14.839);
            public static final double MID_RADIANS = Units.degreesToRadians(115);
            public static final double HIGH_RADIANS = Units.degreesToRadians(49); // TODO: Change this
            public static final double DOUBLE_SUBSTATION_RADIANS = Units.degreesToRadians(69.5);
    
            public static final double GROUND_CONE_RADIANS = Units.degreesToRadians(342.1);
        }
    }

    public interface INDICATOR_VALUES {
        public static final double POSITION_UNKNOWN = -1.0;
        public static final double POSITION_NOT_SET = -1.1;
    }
}