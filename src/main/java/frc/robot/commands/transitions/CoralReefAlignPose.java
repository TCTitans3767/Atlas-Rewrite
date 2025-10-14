package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.DashboardButtonBox;
import frc.robot.Robot;
import frc.robot.commands.Intake.SetIntakePosition;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.manipulator.SetManipulatorWheelSpeed;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.DrivetrainPublisher;
import frc.robot.utils.RobotTransitions;
import frc.robot.utils.Transition;
import frc.robot.utils.Utils.ReefPosition;

@Transition
public class CoralReefAlignPose extends SequentialCommandGroup{

        public class L1 extends SequentialCommandGroup{
        public L1() {
            addCommands(
                new ParallelCommandGroup(
                    new SetIntakePosition(0.22),
                    new SetManipulatorWheelSpeed(0),
                    new SetArmAngle(0.08),
                    new SetElevatorPosition(0.34)
                )
            );
        }
    }

    public class L2 extends ParallelCommandGroup{
        public L2() {
            addCommands(
                new SetManipulatorWheelSpeed(-0.05),
                new SetElevatorPosition(Constants.L2Measurements.elevtaorHeight).withTimeout(0.4),
                new SetArmAngle(Constants.L2Measurements.armAngle)
            );
        }
    }

    public class L3 extends ParallelCommandGroup{
        public L3() {
            addCommands(
                new SetManipulatorWheelSpeed(-0.05),
                new SetElevatorPosition(Constants.L3Measurements.elevtaorHeight).withTimeout(0.4),
                new SetArmAngle(Constants.L3Measurements.armAngle)
            );
        }
    }

    // public class L4 extends ParallelCommandGroup{
    //     public L4() {
    //         addCommands(
    //             new SetManipulatorWheelSpeed(-0.05),
    //             new SetElevatorPosition(Constants.L4Measurements.elevtaorHeight).withTimeout(0.4),
    //             new SetArmAngle(Constants.L4Measurements.armAngle)
    //         );
    //     }
    // }

    public class L4 extends ParallelCommandGroup{
        public L4() {
            addCommands(
                new SetManipulatorWheelSpeed(-0.05),
                new SetElevatorPosition(Constants.L4Measurements.elevtaorHeight).withTimeout(0.4),
                new SetArmAngle(Constants.L4Measurements.armAngle)
            );
        }
    }

    

    private Map<String, Command> commandMap = new HashMap<String, Command>();

    Command leftReefAlign = new InstantCommand(() -> {
        RobotControl.setDriveModeCommand(RobotControl.alignWithLeftReef);});
    Command rightReefAlign = new InstantCommand(() -> {RobotControl.setDriveModeCommand(RobotControl.alignWithRightReef);});
    
    public CoralReefAlignPose() {

        commandMap.put("1", new L1());
        commandMap.put("2", new L2());
        commandMap.put("3", new L3());
        commandMap.put("4", new L4());

        addCommands(
            new ConditionalCommand(leftReefAlign, rightReefAlign, CoralReefAlignPose::isLeftBranchSelected),
            // new ParallelCommandGroup(
            //     new ConditionalCommand(new SetElevatorPosition(Constants.L2Measurements.elevtaorHeight), new SetElevatorPosition(Constants.L3Measurements.elevtaorHeight), TriggerBoard::isL2Selected),
            //     new SetArmAngle(0.2)
            // ),
            new SelectCommand<String>(commandMap, DashboardButtonBox::getSelectedLevelString),
            new WaitUntilCommand(CoralReefAlignPose::isAlignCommandFinsihed).withTimeout(1),
            new InstantCommand(() -> DrivetrainPublisher.setSuppliers(() -> 0, () -> 0, () -> 0)),
            // new ConditionalCommand(new SetArmAngle(Constants.L4Measurements.armAngle), Commands.none(), TriggerBoard::isL4Selected),
            // new SelectCommand<String>(commandMap, DashboardButtonBox::getSelectedLevelString),
            new InstantCommand(() -> {RobotControl.setDriveModeCommand(RobotControl.slowControllerDrive);}),
            new InstantCommand(() -> {RobotControl.setCurrentMode(RobotTransitions.scoreCoralPose);})
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    public static boolean isLeftBranchSelected() {
        return (DashboardButtonBox.getSelectedReefBranch() == ReefPosition.A || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.C ||DashboardButtonBox.getSelectedReefBranch() == ReefPosition.E || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.G || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.I || DashboardButtonBox.getSelectedReefBranch() == ReefPosition.K);
    }

    public static boolean isAlignCommandFinsihed() {
        return CoralReefAlignPose.isLeftBranchSelected() ? RobotControl.alignWithLeftReef.isFinished() : RobotControl.alignWithRightReef.isFinished();
    }
}
