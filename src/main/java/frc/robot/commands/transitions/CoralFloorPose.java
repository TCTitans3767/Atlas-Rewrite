package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.Intake.SetIntakePosition;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.manipulator.SetManipulatorWheelSpeed;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.RobotStates;
import frc.robot.utils.Transition;

@Transition
public class CoralFloorPose extends SequentialCommandGroup{
    
    public CoralFloorPose() {

        addCommands(
            new ParallelCommandGroup(
                new SetIntakePosition(-0.12),
                new SetArmAngle(-0.245),
                new SetElevatorPosition(0.02).withTimeout(0.4),
                new SetManipulatorWheelSpeed(-0.25)
            ),
            new InstantCommand(() -> RobotControl.setCurrentMode(RobotStates.coralFloor))
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
