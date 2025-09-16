package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.SetIntakePosition;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.manipulator.SetManipulatorWheelSpeed;

public class PrepL2Auton extends SequentialCommandGroup{
    
    public PrepL2Auton() {
        addCommands(
            new SetManipulatorWheelSpeed(-0.05),
            new SetArmAngle(-0.44),
            new SetElevatorPosition(0.02),
            new SetIntakePosition(0.32)
        );
    }

}
