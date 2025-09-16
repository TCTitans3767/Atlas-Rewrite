package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ManipulatorIOHardware implements ManipulatorIO{

    private final TalonFX motor;
    private final TalonFXConfiguration config;

    private final Slot0Configs slot0Config;
    private final MotionMagicConfigs motionMagicConfig;

    private final CANrange manipulatorSensor;
    private final CANrangeConfiguration manipulatorSensorConfig;

    private final StatusSignal<Voltage> manipulatorAppliedVolts;
    private final StatusSignal<Current> manipulatorCurrent;
    private final StatusSignal<Boolean> isGamePieceInManipulator;

    public ManipulatorIOHardware() {
        // Motor setup
        motor = new TalonFX(frc.robot.Constants.Manipulator.motorID);
        config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.Feedback.SensorToMechanismRatio = Constants.Manipulator.conversionFactor;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Slot 0 PID setup
        slot0Config = new Slot0Configs();
        slot0Config.kP = Constants.Manipulator.kP;
        slot0Config.kI = Constants.Manipulator.kI;
        slot0Config.kD = Constants.Manipulator.kD;
        slot0Config.kG = Constants.Manipulator.kG;
        slot0Config.kV = Constants.Manipulator.kV;
        slot0Config.kS = Constants.Manipulator.kS;
        slot0Config.GravityType = GravityTypeValue.Arm_Cosine;

        // Motion Magic setup
        motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = Constants.Manipulator.maxVelocity;
        motionMagicConfig.MotionMagicAcceleration = Constants.Manipulator.maxAcceleration;

        // Set the configurations
        motor.getConfigurator().apply(config);
        motor.getConfigurator().apply(slot0Config);
        motor.getConfigurator().apply(motionMagicConfig);
        motor.setNeutralMode(NeutralModeValue.Brake);

        manipulatorSensor = new CANrange(Constants.Manipulator.sensorID);
        manipulatorSensorConfig = new CANrangeConfiguration();
        manipulatorSensorConfig.ProximityParams.ProximityThreshold = Constants.Manipulator.detectionRange;
        manipulatorSensorConfig.ProximityParams.ProximityHysteresis = Constants.Manipulator.sensorDebounce;
        manipulatorSensorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        manipulatorSensor.getConfigurator().apply(manipulatorSensorConfig);

        manipulatorAppliedVolts = motor.getMotorVoltage();
        manipulatorCurrent = motor.getTorqueCurrent();
        isGamePieceInManipulator = manipulatorSensor.getIsDetected();

        BaseStatusSignal.setUpdateFrequencyForAll(Constants.Manipulator.ODOMETRY_FREQUENCY, manipulatorCurrent, manipulatorAppliedVolts, isGamePieceInManipulator);
    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.manipulatorAppliedVolts = manipulatorAppliedVolts.getValueAsDouble();
        inputs.manipulatorCurrent = manipulatorCurrent.getValueAsDouble();
        inputs.manipulatorHasCoral = isGamePieceInManipulator.getValue();
        inputs.manipulatorSetSpeed = motor.get();
        inputs.manipulatorHasAlgae = inputs.manipulatorCurrent > 70;
    }

    @Override
    public void setManipulatorWheelSpeed(double speed) {
        motor.set(speed);
    }
}
