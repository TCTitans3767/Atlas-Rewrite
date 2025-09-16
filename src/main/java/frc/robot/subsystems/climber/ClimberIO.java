package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double climberRotations = 0.0;
        public double climberVelocityRotationsPerSec = 0.0;
        public double climberAppliedVolts = 0.0;
        public double climberCurrent = 0.0;
        public boolean climberAtSetpoint = false;
        public double climberSetpoint = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setClimberPosition(double position) {}

    public default void setClimberSpeed(double speed) {}
}
