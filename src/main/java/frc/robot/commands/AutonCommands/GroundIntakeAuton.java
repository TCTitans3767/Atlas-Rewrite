package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.Intake.SetIntakePosition;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.manipulator.SetManipulatorWheelSpeed;

public class GroundIntakeAuton extends ParallelCommandGroup{
    
    public GroundIntakeAuton() {
        addCommands(
            new SetIntakePosition(-0.11),
            new SetElevatorPosition(0.025),
            new SetManipulatorWheelSpeed(-0.2),
            new SetArmAngle(-0.03)
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);

    }

}
