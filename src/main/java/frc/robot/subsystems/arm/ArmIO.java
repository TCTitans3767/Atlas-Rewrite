package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    @AutoLog
    public static class ArmIOInputs {
        public double armPositionRotations = 0.0;
        public double armVelocityRotationsPerSec = 0.0;
        public double armAppliedVolts = 0.0;
        public double armCurrent = 0.0;
        public boolean isArmAtSetpoint = false;
        public double motionMagicError = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setArmPosition(double position) {}

    public default void setArmSpeed(double speed) {}
}
