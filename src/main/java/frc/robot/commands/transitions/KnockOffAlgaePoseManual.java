package frc.robot.commands.transitions;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.DrivetrainPublisher;

public class KnockOffAlgaePoseManual extends SequentialCommandGroup{
    
    public KnockOffAlgaePoseManual() {

        addCommands(
            new ParallelCommandGroup(
                new SetArmAngle(-0.02),
                new SetElevatorPosition(1.05)
            ),
            new WaitUntilCommand(() -> Robot.joystick.a().getAsBoolean()),
            new InstantCommand(() -> Robot.elevator.setSpeed(-0.4)),
            new WaitCommand(0.2),
            new ParallelRaceGroup(
                new WaitUntilCommand(KnockOffAlgaePose::isElevatorAtPosition),
                new WaitUntilCommand(KnockOffAlgaePose::isManipulatorTouchingAlgae)
            ),
            new InstantCommand(() -> {
                Robot.elevator.setSpeed(0);
                DrivetrainPublisher.setInFieldCentricSupplier(() -> false);
                DrivetrainPublisher.setXVelocitySupplier(() -> -0.5, true);
            }),
            new WaitCommand(0.6),
            new InstantCommand(() -> {DrivetrainPublisher.setXVelocitySupplier(() -> 0, true); RobotControl.setDriveModeCommand(RobotControl.controllerDrive);}),
            new InstantCommand(() -> {RobotControl.setCurrentMode(RobotControl.coralFloorPose); Robot.manipulator.setSpeed(0);})
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    public static boolean isAlignCommandFinsihed() {
        return RobotControl.alignWithAlgae.isFinished();
    }

    public static boolean isElevatorAtPosition() {
        return Robot.elevator.getHeightMeters() <= 0.05;
    }

    public static boolean isManipulatorTouchingAlgae() {
        return (Robot.arm.getMotionMagicError() > 0.01 || Robot.arm.getMotionMagicError() < -0.01) && Robot.elevator.getHeightMeters() <= 0.85;
    }

}
