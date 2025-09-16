package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
    private final ManipulatorIO io;
    private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();

    public Manipulator(ManipulatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Manipulator", inputs);
    }

    public void setSpeed(double speed) {
        io.setManipulatorWheelSpeed(speed);
    }

    public boolean hasGamePiece() {
        return inputs.manipulatorHasCoral;
    }

    public boolean hasAlgae() {
        return inputs.manipulatorHasAlgae;
    }

    public double getTourque() {
        return inputs.manipulatorCurrent;
    }

}
