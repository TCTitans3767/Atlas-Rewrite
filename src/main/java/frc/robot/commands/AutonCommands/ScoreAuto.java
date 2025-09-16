package frc.robot.commands.AutonCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.commands.manipulator.SetManipulatorWheelSpeed;
import frc.robot.utils.DrivetrainPublisher;

public class ScoreAuto extends SequentialCommandGroup{
    
    public ScoreAuto() {
        addCommands(
            new SetManipulatorWheelSpeed(0.5),
            new WaitUntilCommand(() -> !TriggerBoard.isCoralInManipulator()),
            Robot.drivetrain.runOnce(() -> {
                DrivetrainPublisher.setInFieldCentricSupplier(() -> false);
                DrivetrainPublisher.setXVelocitySupplier(() -> -0.25, true);
            }),
            new WaitCommand(0.35),
            Robot.drivetrain.runOnce(() -> {
                DrivetrainPublisher.setInFieldCentricSupplier(() -> true);
                DrivetrainPublisher.setXVelocitySupplier(() -> 0, true);
            })
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);

    }

}
