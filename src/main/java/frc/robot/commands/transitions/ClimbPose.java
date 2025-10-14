package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.Intake.SetIntakePosition;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.RobotTransitions;
import frc.robot.utils.State;
import frc.robot.utils.Transition;

@Transition
public class ClimbPose extends SequentialCommandGroup{
    
    public ClimbPose() {

        addCommands(
            new SetIntakePosition(-0.06),
            new SetArmAngle(-0.5),
            new SetElevatorPosition(0.021),
            new InstantCommand(() -> RobotControl.setCurrentMode(RobotTransitions.deployClimberPose))
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
