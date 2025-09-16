package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.robotControl.RobotControl;

public class SetDriveModeCommand extends Command{

    Command newDriveCommand;
    
    public SetDriveModeCommand(Command newDriveCommand) {
        this.newDriveCommand = newDriveCommand;
    }

    @Override
    public void initialize() {
        RobotControl.setDriveModeCommand(newDriveCommand);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
