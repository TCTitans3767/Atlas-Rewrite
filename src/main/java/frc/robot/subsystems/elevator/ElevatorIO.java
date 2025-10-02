package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public double elevatorPositionMeters = 0.0;
        public double elevatorVelocityMetersPerSec = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double elevatorCurrent= 0.0;
        public double elevatorSetHeightMeters = 0.0;
        public boolean elevatorAtSetpoint = false;
        public double elevatorError = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setHeight(double meters) {}

    public default void setSpeed(double speed) {}
}
