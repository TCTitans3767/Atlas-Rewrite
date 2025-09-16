package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io) {
       this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void setPosition(double meters) {
        io.setHeight(meters);
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }

    public double getHeightMeters() {
        return inputs.elevatorPositionMeters;
    }

    public boolean isAtPosition() {
        return inputs.elevatorAtSetpoint;
    }
}
