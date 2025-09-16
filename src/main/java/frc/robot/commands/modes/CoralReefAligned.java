package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.robotControl.RobotControl;

public class CoralReefAligned extends Command{

    public CoralReefAligned() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {
        if (TriggerBoard.isCoralButtonPressed()) {
            RobotControl.setCurrentMode(RobotControl.coralReefPose);
            return;
        }

        if (TriggerBoard.isCoralOverrideButtonPressed()) {
            RobotControl.setCurrentMode(RobotControl.transitPose);
            return;
        }
    }

}