package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.utils.DrivetrainPublisher;

public class ControllerDrive extends InstantCommand {
    public ControllerDrive() {
        super(() -> {
            DrivetrainPublisher.setInFieldCentricSupplier(() -> true);
            DrivetrainPublisher.setSuppliers(Robot.joystick::getLeftY, Robot.joystick::getLeftX, Robot.joystick::getRightX);
            DrivetrainPublisher.setAcceptInputsSupplier(() -> true);
        });

    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
