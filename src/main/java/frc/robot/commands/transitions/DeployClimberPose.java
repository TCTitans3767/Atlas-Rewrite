package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.climb.SetClimberRotations;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.RobotStates;
import frc.robot.utils.RobotTransitions;
import frc.robot.utils.Transition;

@Transition
public class DeployClimberPose extends SequentialCommandGroup{
    
    public DeployClimberPose() {

        addCommands(
            new SetClimberRotations(114),
            new InstantCommand(() -> RobotControl.setDriveModeCommand(RobotControl.slowControllerDrive)),
            new InstantCommand(() -> RobotControl.setCurrentMode(RobotStates.finalClimb))
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }
    
}
