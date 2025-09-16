package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Intake.SetIntakePosition;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.manipulator.SetManipulatorWheelSpeed;

public class PrepL4Auton extends ParallelCommandGroup{
    
    public PrepL4Auton() {
        addCommands(
            new SetManipulatorWheelSpeed(-0.05),
            new SetArmAngle(-0.43),
            new SetElevatorPosition(1.1),
            new SetIntakePosition(0.32)
        );
    }

}
