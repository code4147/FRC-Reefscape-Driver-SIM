// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Isaac Is Cool
package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.AutoLog;
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean isMotorConnected = false;

        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVoltage = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }
    public default void updateInputs(ElevatorIOInputs inputs) {}
    public default void setVoltage(double volts) {}
    public default void setPosition(double positionMeters) {}
    public default void stop() {}
    public default void setGains(double p, double i, double d, double s, double g, double v, double a) {}
    public default void setMotionMagicConstraints(double maxVelocity, double maxAcceleration) {}
    public default void setBrakeMode(boolean enableBrake) {}
    public default void resetPosition() {}
}
