package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.Intake.SetIntakeWheelPower;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.RobotTransitions;
import frc.robot.utils.Transition;

@Transition
public class EjectAlgaePose extends SequentialCommandGroup{
    
    public EjectAlgaePose() {

        addCommands(
            new SetIntakeWheelPower(-0.7),
            new InstantCommand(() -> {
                Robot.intake.setWheelPower(0);
            }),
            new WaitCommand(0.25),
            new InstantCommand(() -> RobotControl.setCurrentMode(RobotTransitions.transitPose))
        );

        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

}
