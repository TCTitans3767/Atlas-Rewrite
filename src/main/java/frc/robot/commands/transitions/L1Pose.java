package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.SetIntakePosition;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.RobotStates;
import frc.robot.utils.Transition;

@Transition
public class L1Pose extends SequentialCommandGroup{

    public L1Pose() {
        addCommands(
            new SetIntakePosition(0.2),
            new InstantCommand(() -> RobotControl.setCurrentMode(RobotStates.l1))
        );
    }

}
