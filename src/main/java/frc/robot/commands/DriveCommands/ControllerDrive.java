package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.DrivetrainPublisher;

public class ControllerDrive extends InstantCommand {
    public ControllerDrive() {
        super(() -> {
            DrivetrainPublisher.instance.setSuppliers(() -> 0, () -> 0, () -> 0);
            DrivetrainPublisher.instance.setAcceptInputsSupplier(() -> true);
        });

    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
