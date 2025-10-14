package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Intake.SetIntakePosition;
import frc.robot.commands.Intake.SetIntakeWheelSpeed;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.manipulator.SetManipulatorWheelSpeed;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.RobotStates;
import frc.robot.utils.Transition;

@Transition
public class ResetPose extends SequentialCommandGroup{
    
    public ResetPose() {

        addCommands(
            new InstantCommand(() -> {
                RobotControl.setDriveModeCommand(RobotControl.controllerDrive);}).ignoringDisable(true),
            new SetManipulatorWheelSpeed(0),
            new SetIntakePosition(0).withTimeout(3),
            new SetIntakeWheelSpeed(-60),
            new SetArmAngle(0.05),
            new SetElevatorPosition(0.5),
            new SetArmAngle(-0.122),
            new SetIntakePosition(Constants.Intake.pivotStowPosition).withTimeout(3),
            new InstantCommand(() -> {Robot.intake.resetWheelSpeed(); RobotControl.setCurrentMode(RobotStates.transit);}).ignoringDisable(true)
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
