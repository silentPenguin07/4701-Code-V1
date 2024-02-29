package frc.robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import entechlib.commands.EntechCommand;
import frc.robot.commands.GyroReset;
import frc.robot.robot_subsystems.ArmSubsystem;
import frc.robot.robot_subsystems.DriveSubsystem;

public class CommandFactory{
    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;

    public CommandFactory(RobotContainer robotContiner)
    {
        this.driveSubsystem = robotContiner.getDriveSubsystem();
        this.armSubsystem = robotContiner.getArmSubsystem();
    }

    public EntechCommand gyroResetCommand() {
        return new GyroReset(driveSubsystem);
    }
}