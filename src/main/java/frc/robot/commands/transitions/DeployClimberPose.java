package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.climb.SetClimberRotations;
import frc.robot.subsystems.robotControl.RobotControl;

public class DeployClimberPose extends SequentialCommandGroup{
    
    public DeployClimberPose() {

        addCommands(
            new SetClimberRotations(114),
            new InstantCommand(() -> RobotControl.setDriveModeCommand(RobotControl.slowControllerDrive)),
            new InstantCommand(() -> RobotControl.setCurrentMode(RobotControl.finalClimb))
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }
    
}
