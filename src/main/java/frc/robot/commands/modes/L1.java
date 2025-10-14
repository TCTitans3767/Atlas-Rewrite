package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.RobotTransitions;
import frc.robot.utils.State;

@State
public class L1 extends Command {

    public L1() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {

        if (TriggerBoard.isCoralInManipulator()) {
            RobotControl.setCurrentMode(RobotTransitions.transitPose);
        }

        if (TriggerBoard.isCoralButtonPressed()) {
            RobotControl.setCurrentMode(RobotTransitions.scoreCoralPose);
        }

    }

}
