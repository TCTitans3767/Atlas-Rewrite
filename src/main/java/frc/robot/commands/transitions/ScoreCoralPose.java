package frc.robot.commands.transitions;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.DashboardButtonBox;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.DrivetrainPublisher;

public class ScoreCoralPose extends SequentialCommandGroup {

    private class scoreAndTransit extends SequentialCommandGroup {
        public scoreAndTransit() {
            addCommands(
                    new InstantCommand(() -> {
                        if (TriggerBoard.isL1Selected() && !DriverStation.isAutonomousEnabled()) {
                            Robot.elevator.setPosition(0.02);
                            DrivetrainPublisher.setInFieldCentricSupplier(() -> false);
                            DrivetrainPublisher.setXVelocitySupplier(() -> 1.8, true);
                        } else {
                            DrivetrainPublisher.setInFieldCentricSupplier(() -> false);
                            DrivetrainPublisher.setXVelocitySupplier(() -> -0.5, true);
                        }

                    }),
                    new WaitCommand(0.25),
                    new InstantCommand(() -> {
                        if (!DriverStation.isAutonomousEnabled()) {
                            RobotControl.setDriveModeCommand(RobotControl.controllerDrive);
                        } else {
                            DrivetrainPublisher.setXVelocitySupplier(() -> 0, true);
                        }
                    }),
                    new InstantCommand(() -> {
                        Robot.manipulator.setSpeed(0);
                        RobotControl.setCurrentMode(RobotControl.coralFloorPose);
                    }));
        }
    }

    private double timer = 0;

    // private Map<String, Command> commandMap = new HashMap<String, Command>();

    public ScoreCoralPose() {

        // commandMap.put("4", new SetArmAngle(Constants.L4Measurements.armAngle));
        // commandMap.put("3", new SetArmAngle(Constants.L3Measurements.armAngle));
        // commandMap.put("2", new SetArmAngle(Constants.L2Measurements.armAngle));

        addCommands(
                // new SelectCommand<String>(commandMap,
                // DashboardButtonBox::getSelectedLevelString),
                new InstantCommand(() -> {
                    if (TriggerBoard.isL1Selected()) {
                        Robot.intake.scoreL1();
                    } else if (TriggerBoard.isL4Selected()) {
                        Robot.manipulator.setSpeed(0.4);
                    } else {
                        Robot.manipulator.setSpeed(0.25);
                    }
                }),
                new ConditionalCommand(new WaitCommand(1.5), new WaitCommand(.15), TriggerBoard::isL1Selected),
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new SetArmAngle(0.17),
                                new SetElevatorPosition(1.1)),
                        Commands.none(),
                        TriggerBoard::isL4Selected),
                new ConditionalCommand(
                        new InstantCommand(() -> {
                            Robot.manipulator.setSpeed(0);
                            RobotControl.setCurrentMode(RobotControl.knockOffAlgaePose);
                        }),
                        new scoreAndTransit(),
                        DashboardButtonBox::isAlgaeKnockoffOn));

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
