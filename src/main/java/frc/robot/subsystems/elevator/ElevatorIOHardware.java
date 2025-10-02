package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ElevatorIOHardware implements ElevatorIO{
    private final TalonFX rightMotor, leftMotor;
    private final TalonFXConfiguration leftConfig, rightConfig;

    private final Slot0Configs slot0Config;
    private final MotionMagicConfigs motionMagicConfig;
    private final MotionMagicVoltage motionMagicRequest;

    private final StatusSignal<Angle> elevatorHeight;
    private final StatusSignal<AngularVelocity> elevatorVelocity;
    private final StatusSignal<Voltage> elevatorAppliedVolts;
    private final StatusSignal<Current> elevatorCurrent;
    private double setHeight = 0.0;

    public ElevatorIOHardware() {
        // Motor basic setup
        leftMotor = new TalonFX(Constants.Elevator.leftMotorID);
        leftConfig = new TalonFXConfiguration();
        leftConfig.Feedback.SensorToMechanismRatio = Constants.Elevator.conversionFactor;
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Elevator.metersMax * Constants.Elevator.RotationsPerMeter;
        leftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Elevator.metersMin * Constants.Elevator.RotationsPerMeter;
        leftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        leftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // Slot 0 PID setup
        slot0Config = new Slot0Configs();
        slot0Config.kP = Constants.Elevator.kP;
        slot0Config.kI = Constants.Elevator.kI;
        slot0Config.kD = Constants.Elevator.kD;
        slot0Config.kG = Constants.Elevator.kG;
        slot0Config.kV = Constants.Elevator.kV;
        slot0Config.kS = Constants.Elevator.kS;
        slot0Config.GravityType = GravityTypeValue.Elevator_Static;

        // Motion Magic setup
        motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = Constants.Elevator.maxVelocity;
        motionMagicConfig.MotionMagicAcceleration = Constants.Elevator.maxAcceleration;

        // Set the configurations
        leftMotor.getConfigurator().apply(leftConfig);
        leftMotor.getConfigurator().apply(slot0Config);
        leftMotor.getConfigurator().apply(motionMagicConfig);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);

        rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor = new TalonFX(Constants.Elevator.rightMotorID);
        rightMotor.getConfigurator().apply(rightConfig);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setControl(new Follower(Constants.Elevator.leftMotorID, true));

        motionMagicRequest = new MotionMagicVoltage(0);

        elevatorHeight = leftMotor.getPosition();
        elevatorVelocity = leftMotor.getVelocity();
        elevatorAppliedVolts = leftMotor.getSupplyVoltage();
        elevatorCurrent = leftMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            Constants.Elevator.ODOMETRY_FREQUENCY, elevatorHeight, elevatorVelocity, elevatorAppliedVolts, elevatorCurrent);

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(elevatorHeight, elevatorVelocity, elevatorAppliedVolts, elevatorCurrent);
        inputs.elevatorPositionMeters = elevatorHeight.getValueAsDouble() / Constants.Elevator.RotationsPerMeter;
        inputs.elevatorVelocityMetersPerSec = elevatorVelocity.getValueAsDouble();
        inputs.elevatorAppliedVolts = elevatorAppliedVolts.getValueAsDouble();
        inputs.elevatorCurrent = elevatorCurrent.getValueAsDouble();
        inputs.elevatorSetHeightMeters = motionMagicRequest.Position;
        inputs.elevatorAtSetpoint = leftMotor.getClosedLoopError().getValueAsDouble() < Constants.Elevator.errorTolerance;
        inputs.elevatorError = leftMotor.getClosedLoopError().getValueAsDouble();
    }

    @Override
    public void setHeight(double meters) {
         leftMotor.setControl(motionMagicRequest.withPosition(meters));
    }

    @Override
    public void setSpeed(double speed) {
        leftMotor.set(speed);
    }
}
