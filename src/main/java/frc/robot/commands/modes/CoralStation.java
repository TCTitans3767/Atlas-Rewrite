package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TriggerBoard;
import frc.robot.subsystems.robotControl.RobotControl;
import frc.robot.utils.RobotTransitions;
import frc.robot.utils.State;

@State
public class CoralStation extends Command{
    
    public CoralStation() {
        addRequirements(Robot.arm, Robot.climber, Robot.intake, Robot.manipulator, Robot.elevator);
    }

    @Override
    public void execute() {

        if (TriggerBoard.isL1Selected() && Robot.manipulator.getTourque() > 40) {
            RobotControl.setCurrentMode(null);
        }

        if (TriggerBoard.isCoralInManipulator()) {
            RobotControl.setCurrentMode(RobotTransitions.coralRecievedPose);
            return;
        }

        // if (TriggerBoard.isCoralButtonPressed() && TriggerBoard.isNearCoralStation()) {
        //     Robot.robotMode.setCurrentMode(RobotMode.coralStationAlignPose);
        // }
    }

}
