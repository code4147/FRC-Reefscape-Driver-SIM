// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX kMotor = new TalonFX(ElevatorConstants.kMotorID);

    private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    // Motor data we wish to log
    private StatusSignal<Angle> positionRotations;
    private StatusSignal<AngularVelocity> velocityRotationsPerSec;
    private StatusSignal<Voltage> appliedVolts;
    private StatusSignal<Current> supplyCurrentAmps;
    private StatusSignal<Current> statorCurrentAmps;
    private StatusSignal<Temperature> temperatureCelsius;

    // Control modes
    private final VoltageOut kVoltageControl = new VoltageOut(0.0);
    private final MotionMagicVoltage kPositionControl = new MotionMagicVoltage(0.0);

    public ElevatorIOTalonFX() {
        // Apply configurations
        motorConfiguration.Slot0.kP = ElevatorConstants.kElevatorGains.kP();
        motorConfiguration.Slot0.kI = ElevatorConstants.kElevatorGains.kI();
        motorConfiguration.Slot0.kD = ElevatorConstants.kElevatorGains.kD();
        motorConfiguration.Slot0.kS = ElevatorConstants.kElevatorGains.kS();
        motorConfiguration.Slot0.kV = ElevatorConstants.kElevatorGains.kV();
        motorConfiguration.Slot0.kA = ElevatorConstants.kElevatorGains.kA();
        motorConfiguration.Slot0.kG = ElevatorConstants.kElevatorGains.kG();
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity =
                ElevatorConstants.kElevatorGains.kMaxVelocityMetersPerSecond();
        motorConfiguration.MotionMagic.MotionMagicAcceleration =
                ElevatorConstants.kElevatorGains.kMaxAccelerationMetersPerSecondSquared();
        motorConfiguration.MotionMagic.MotionMagicJerk = ElevatorConstants.kElevatorGains.kJerkMetersPerSecondCubed();

        motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable =
                ElevatorConstants.kMotorConfiguration.kEnableSupplyCurrentLimit();
        motorConfiguration.CurrentLimits.SupplyCurrentLimit =
                ElevatorConstants.kMotorConfiguration.kSupplyCurrentLimitAmps();
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable =
                ElevatorConstants.kMotorConfiguration.kEnableStatorCurrentLimit();
        motorConfiguration.CurrentLimits.StatorCurrentLimit =
                ElevatorConstants.kMotorConfiguration.kStatorCurrentLimitAmps();

        motorConfiguration.MotorOutput.NeutralMode = ElevatorConstants.kMotorConfiguration.kNeutralMode();
        motorConfiguration.MotorOutput.Inverted = ElevatorConstants.kMotorConfiguration.kInvert()
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;

        // Reset position on startup
        kMotor.setPosition(0.0);

        motorConfiguration.Feedback.SensorToMechanismRatio =
                // Rotations -> Linear distance (Circumfrence of the drum over the gearing)
                (ElevatorConstants.kDrumCircumferenceMeters / ElevatorConstants.kGearing);
        // Rotor sensor is the built-in sensor
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        kMotor.getConfigurator().apply(motorConfiguration, 1.0);

        // Get status signals from the motor controller
        positionRotations = kMotor.getPosition();
        velocityRotationsPerSec = kMotor.getVelocity();
        appliedVolts = kMotor.getMotorVoltage();
        supplyCurrentAmps = kMotor.getSupplyCurrent();
        statorCurrentAmps = kMotor.getStatorCurrent();
        temperatureCelsius = kMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                ElevatorConstants.kStatusSignalUpdateFrequencyHz,
                positionRotations,
                velocityRotationsPerSec,
                appliedVolts,
                supplyCurrentAmps,
                supplyCurrentAmps,
                statorCurrentAmps,
                temperatureCelsius);

        // Optimize the CANBus utilization by explicitly telling all CAN signals we
        // are not using to simply not be sent over the CANBus
        kMotor.optimizeBusUtilization(0.0, 1.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.isMotorConnected = BaseStatusSignal.refreshAll(
                        positionRotations,
                        velocityRotationsPerSec,
                        appliedVolts,
                        supplyCurrentAmps,
                        supplyCurrentAmps,
                        statorCurrentAmps,
                        temperatureCelsius)
                .isOK();

        inputs.positionMeters = positionRotations.getValueAsDouble();
        // TODO Double check that this is reporting velocity correctly, it may be reporting it in meters but idk lol
        inputs.velocityMetersPerSec = rotationsToMeters(
                velocityRotationsPerSec.getValueAsDouble(),
                ElevatorConstants.kDrumCircumferenceMeters,
                ElevatorConstants.kGearing);
        inputs.appliedVoltage = appliedVolts.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        kMotor.setControl(kVoltageControl.withOutput(volts));
    }

    @Override
    public void setPosition(double positionMeters) {
        kMotor.setControl(kPositionControl.withPosition(positionMeters).withSlot(0));
    }

    @Override
    public void stop() {
        kMotor.setControl(new NeutralOut());
    }

    @Override
    public void resetPosition() {
        kMotor.setPosition(0.0);
    }

    @Override
    public void setGains(double p, double i, double d, double s, double g, double v, double a) {
        var slotConfiguration = new Slot0Configs();

        slotConfiguration.kP = p;
        slotConfiguration.kI = i;
        slotConfiguration.kD = d;
        slotConfiguration.kS = s;
        slotConfiguration.kG = g;
        slotConfiguration.kV = v;
        slotConfiguration.kA = a;

        kMotor.getConfigurator().apply((slotConfiguration));
    }

    @Override
    public void setMotionMagicConstraints(double maxVelocity, double maxAcceleration) {
        var motionMagicConfiguration = new MotionMagicConfigs();

        motionMagicConfiguration.MotionMagicCruiseVelocity = maxVelocity;
        motionMagicConfiguration.MotionMagicAcceleration = maxAcceleration;
        motionMagicConfiguration.MotionMagicJerk = 10.0 * maxAcceleration;

        kMotor.getConfigurator().apply(motionMagicConfiguration);
    }

    @Override
    public void setBrakeMode(boolean enableBrake) {
        kMotor.setNeutralMode(enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    private double rotationsToMeters(double rotations, double circumference, double gearRatio) {
        double wheelRevolutions = rotations / gearRatio;
        double wheelDistance = wheelRevolutions * circumference;
        return wheelDistance;
    }
}
