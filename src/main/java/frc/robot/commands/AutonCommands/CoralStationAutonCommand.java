package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.commands.Intake.SetIntakePosition;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.manipulator.SetManipulatorWheelSpeed;

public class CoralStationAutonCommand extends SequentialCommandGroup{
    
    public CoralStationAutonCommand() {
        
        addCommands(
            new SetArmAngle(-0.128),
            new ParallelCommandGroup(
                new SetElevatorPosition(0.5),
                new SetArmAngle(0.128),
                new SetManipulatorWheelSpeed(-0.3)
            ),
            new SetIntakePosition(0.32),
            new WaitUntilCommand(TriggerBoard::isCoralInManipulator),
            new SetManipulatorWheelSpeed(0)
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
