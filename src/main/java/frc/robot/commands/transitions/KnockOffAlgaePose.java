package frc.robot.commands.transitions;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.DrivetrainPublisher;
import frc.robot.utils.RobotTransitions;
import frc.robot.utils.Transition;

@Transition
public class KnockOffAlgaePose extends SequentialCommandGroup{

    Command algaeAlign = new InstantCommand(() -> {
        RobotControl.setDriveModeCommand(RobotControl.alignWithAlgae);});

    Command bounceToTransit = new SequentialCommandGroup(
        new InstantCommand(() -> {
            DrivetrainPublisher.setInFieldCentricSupplier(() -> false);
            if (TriggerBoard.isL1Selected()) {
                DrivetrainPublisher.setXVelocitySupplier(() -> 0.55, true);
            } else {
                DrivetrainPublisher.setXVelocitySupplier(() -> -0.35, true);
            }
        }),
        new WaitCommand(0.25),
        new InstantCommand(() -> {if (!DriverStation.isAutonomousEnabled()) {RobotControl.setDriveModeCommand(RobotControl.controllerDrive);} else {DrivetrainPublisher.setXVelocitySupplier(() -> 0, true);}}),
        new InstantCommand(() -> {
            Robot.manipulator.setSpeed(0);
            RobotControl.setCurrentMode(RobotTransitions.coralFloorPose);
        })
    );
    
    public KnockOffAlgaePose() {

        addCommands(
            new ConditionalCommand(algaeAlign, new InstantCommand(() -> RobotControl.setCurrentMode(RobotTransitions.transitPose)), TriggerBoard::isL4Selected),
            new WaitUntilCommand((KnockOffAlgaePose::isAlignCommandFinsihed)).withTimeout(1),
            new ParallelCommandGroup(
                new SetArmAngle(-0.07),
                new SetElevatorPosition(1.1)
            ),
            new InstantCommand(() -> Robot.manipulator.setSpeed(-0.7)),
            new InstantCommand(() -> Robot.elevator.setSpeed(-0.4)),
            new WaitCommand(0.2),
            new ParallelRaceGroup(
                new WaitUntilCommand(KnockOffAlgaePose::isElevatorAtPosition),
                new WaitUntilCommand(KnockOffAlgaePose::isManipulatorTouchingAlgae)
            ),
            new InstantCommand(() -> {
                Robot.elevator.setSpeed(0);
                DrivetrainPublisher.setInFieldCentricSupplier(() -> false);
                DrivetrainPublisher.setXVelocitySupplier(() -> -0.9, true);
            }),
            new WaitCommand(0.6),
            new InstantCommand(() -> {DrivetrainPublisher.setXVelocitySupplier(() -> 0, true); RobotControl.setDriveModeCommand(RobotControl.controllerDrive);}),
            new InstantCommand(() -> {RobotControl.setCurrentMode(RobotTransitions.coralFloorPose); Robot.manipulator.setSpeed(0);})
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    public static boolean isAlignCommandFinsihed() {
        System.out.println("align finished: " + RobotControl.alignWithAlgae.isFinished());
        return RobotControl.alignWithAlgae.isFinished() || TriggerBoard.isCoralButtonPressed();
    }

    public static boolean isElevatorAtPosition() {
        return Robot.elevator.getHeightMeters() <= 0.05;
    }

    public static boolean isManipulatorTouchingAlgae() {
        return (Robot.arm.getMotionMagicError() > 0.015 || Robot.arm.getMotionMagicError() < -0.015) && Robot.elevator.getHeightMeters() <= 0.85;
    }

}
