package frc.robot.OI;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.CommandFactory;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmBackwardCommand;
import frc.robot.commands.ArmForwardCommand;
import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.XCommand;
import frc.robot.robot_subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OperatorInterface {

        private final XboxController driveJoystick = new XboxController(RobotConstants.Ports.CONTROLLER.DRIVER_JOYSTICK);
        private final XboxController armController = new XboxController(RobotConstants.Ports.CONTROLLER.ARM_JOYSTICK);

        public OperatorInterface(CommandFactory commandFactory, RobotContainer robotContainer) {
                

                //TODO: Button numbers need to be changed
                new JoystickButton(driveJoystick, 2).onTrue(commandFactory.gyroResetCommand());
                new JoystickButton(driveJoystick, 3).onTrue(new XCommand());
                robotContainer.getDriveSubsystem()
                                .setDefaultCommand(new DriveCommand(robotContainer.getDriveSubsystem(), driveJoystick));
                

                //new JoystickButton(armController, 2).whileTrue(new ArmRotateCommand(robotContainer.getArmSubsystem(), RobotConstants.ArmConstants.RotationSetpoints.LOW_RADIANS));
                //new JoystickButton(armController, 3).whileTrue(new ArmRotateCommand(robotContainer.getArmSubsystem(), RobotConstants.ArmConstants.RotationSetpoints.HIGH_RADIANS));
          
                new JoystickButton(armController, 3).whileTrue(new ArmForwardCommand(robotContainer.getArmSubsystem()));
                //new JoystickButton(armController, 2).whileTrue(new ArmBackwardCommand(robotContainer.getArmSubsystem()));
                new JoystickButton(armController, 5).whileTrue(new IntakeCommand(robotContainer.getShooterIntakeSubsystem(), true));
                new JoystickButton(armController, 4).whileTrue(new IntakeCommand(robotContainer.getShooterIntakeSubsystem(), false));
                new JoystickButton(armController, 6).whileTrue(new ShootCommand(robotContainer.getShooterIntakeSubsystem(), false));
                new JoystickButton(armController, 9).whileTrue(new ShootCommand(robotContainer.getShooterIntakeSubsystem(), true));
        }
}
