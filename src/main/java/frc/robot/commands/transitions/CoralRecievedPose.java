package frc.robot.commands.transitions;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.DrivetrainPublisher;

public class CoralRecievedPose extends SequentialCommandGroup{
    
    public CoralRecievedPose() {

        addCommands(
                new InstantCommand(() -> {
                    DrivetrainPublisher.setXVelocitySupplier(() -> 0.35, true);
                }),
                new WaitCommand(0.25),
                new InstantCommand(() -> {if (!DriverStation.isAutonomousEnabled()) {RobotControl.setDriveModeCommand(RobotControl.controllerDrive);} else {DrivetrainPublisher.setXVelocitySupplier(() -> 0, true);}}),
                new InstantCommand(() -> {
                    Robot.manipulator.setSpeed(0);
                    RobotControl.setCurrentMode(RobotControl.transitPose);
                })
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
