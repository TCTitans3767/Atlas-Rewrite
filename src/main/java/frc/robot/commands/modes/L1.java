package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.robotControl.RobotControl;

public class L1 extends Command {

    public L1() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {

        if (TriggerBoard.isCoralInManipulator()) {
            RobotControl.setCurrentMode(RobotControl.transitPose);
        }

        if (TriggerBoard.isCoralButtonPressed()) {
            RobotControl.setCurrentMode(RobotControl.scoreCoralPose);
        }

    }

}
