/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Robot;
import frc.utilities.Xbox;

public class ElevatorMMTest {

    TalonSRX elevatorDriverMainR1;
    VictorSPX elevatorDriverR2, elevatorDriverL1, elevatorDriverL2;

    private double KStatic = 0.1;
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double eps = 0;
    double ff = 0;

    private final int accelConst = PpS_to_PpmS(feetToPulses(2.5), 100); // 2.5 ft/s OR pulse per 100ms
    private final int velConst = PpS_to_PpmS(feetToPulses(1.0), 100); // 1ft/s/s OR pulse per 100ms per S

    double commandedHeight;

    private final double HEIGHT_PER_PULSE = (1.562 * 1e-4); // (1/6400)

    // Constants - General
    public final double STARTING_HEIGHT = 0; // ft
    public final double SETPOINT_PREP = 2; // ft
    public final double SETPOINT_BOTTOM = 5.75; // ft
    public final double MAX_HEIGHT_UPPER = 8;

    // Constants - Cargo Placement
    public final double SETPOINT_ROCKET_TOP_CARGO = 5.75; // ft
    public final double SETPOINT_ROCKET_MIDDLE_CARGO = 3; // ft
    public final double SETPOINT_ROCKET_BOTTOM_CARGO = 0; // ft
    public final double SETPOINT_SHIP_CARGO = 0; // ft

    // Constants - Hatch Placement
    public final double SETPOINT_ROCKET_TOP_HATCH = 5.75; // ft
    public final double SETPOINT_ROCKET_MIDDLE_HATCH = 3; // ft
    public final double SETPOINT_ROCKET_BOTTOM_HATCH = 0; // ft

    public ElevatorMMTest(int elevator1CANPort, int elevator2CANPort, int elevator3CANPort, int elevator4CANPort) {
        elevatorDriverMainR1 = new TalonSRX(elevator1CANPort);
		elevatorDriverR2 = new VictorSPX(elevator2CANPort);
		elevatorDriverL1 = new VictorSPX(elevator3CANPort);
		elevatorDriverL2 = new VictorSPX(elevator4CANPort);

        configureSpeedControllers();
        configureLiftDownPID();
    }

    private int feetToPulses(double feet) {
        return (int) Math.round(feet / HEIGHT_PER_PULSE);
    }

    private int PpS_to_PpmS(int pulsePS, int milliseconds) {
        return (int) pulsePS * milliseconds / 1000;
    }

    private void determineSetpoint() {

        // button X(Manipulator to Bottom)
        if (Robot.xboxJoystick.getRawButton(Xbox.POVleft)) {
            commandedHeight = SETPOINT_BOTTOM;
        }

        // Setpoints for Cargo

        // button Y(Deposit Cube in Scale)
        if (Robot.xboxJoystick.getRawButton(Xbox.Y)) {
            commandedHeight = SETPOINT_ROCKET_TOP_CARGO;
        }

        // button A(Deposit Cube in Switch)
        if (Robot.xboxJoystick.getRawButton(Xbox.B)) {
            commandedHeight = SETPOINT_ROCKET_MIDDLE_CARGO;
        }

        // button B(Manipulator to Bottom)
        if (Robot.xboxJoystick.getRawButton(Xbox.A)) {
            commandedHeight = SETPOINT_ROCKET_BOTTOM_CARGO;
        }

        // button B(Manipulator to Bottom)
        if (Robot.xboxJoystick.getRawButton(Xbox.X)) {
            commandedHeight = SETPOINT_SHIP_CARGO;
        }

        // Setpoints for Hatch

        // button Y(Deposit Cube in Scale)
        if (Robot.xboxJoystick.getPOV(Xbox.DPad) == Xbox.POVup) {
            commandedHeight = SETPOINT_ROCKET_TOP_HATCH;
        }

        // button A(Deposit Cube in Switch)
        if (Robot.xboxJoystick.getPOV(Xbox.DPad) == Xbox.POVright) {
            commandedHeight = SETPOINT_ROCKET_MIDDLE_HATCH;
        }

        // button B(Manipulator to Bottom)
        if (Robot.xboxJoystick.getPOV(Xbox.DPad) == Xbox.POVdown) {
            commandedHeight = SETPOINT_ROCKET_BOTTOM_HATCH;
        }

        // Limit commanded height range
        if (commandedHeight > MAX_HEIGHT_UPPER) {
            commandedHeight = MAX_HEIGHT_UPPER;
        } else if (commandedHeight < SETPOINT_BOTTOM) {
            commandedHeight = SETPOINT_BOTTOM;
        }
    }

    public void configureSpeedControllers() {
        this.elevatorDriverMainR1.setControlFramePeriod(ControlFrame.Control_3_General, 20);
        this.elevatorDriverR2.setControlFramePeriod(ControlFrame.Control_3_General, 20);
        this.elevatorDriverR2.follow(this.elevatorDriverMainR1);
        this.elevatorDriverMainR1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        this.elevatorDriverMainR1.setSelectedSensorPosition(0, 0, 0);
        elevatorDriverMainR1.setInverted(true);
        elevatorDriverR2.setInverted(false);
        elevatorDriverMainR1.setSensorPhase(!true);
    }

    public void configureLiftUpPID() {
        this.elevatorDriverMainR1.selectProfileSlot(0, 0);
        this.elevatorDriverMainR1.config_kP(0, kP, 20);
        this.elevatorDriverMainR1.config_kI(0, kI, 20);
        this.elevatorDriverMainR1.config_kD(0, kD, 20);
        this.elevatorDriverMainR1.configAllowableClosedloopError(0, (int) eps, 20);
        this.elevatorDriverMainR1.config_IntegralZone(0, 4096, 20);
        this.elevatorDriverMainR1.config_kF(0, ff, 20);
        this.elevatorDriverMainR1.config_IntegralZone(0, 10000, 20);
        elevatorDriverMainR1.configMotionAcceleration(accelConst, 0); // test values
        elevatorDriverMainR1.configMotionCruiseVelocity(velConst, 0); // test values
    }

    public void configureLiftDownPID() {
        this.elevatorDriverMainR1.selectProfileSlot(1, 0);
        this.elevatorDriverMainR1.config_kP(1, kP, 20);
        this.elevatorDriverMainR1.config_kI(1, kI, 20);
        this.elevatorDriverMainR1.config_kD(1, kD, 20);
        this.elevatorDriverMainR1.configAllowableClosedloopError(1, (int) eps, 20);
        this.elevatorDriverMainR1.config_IntegralZone(1, 6000, 20);
        this.elevatorDriverMainR1.config_kF(1, ff, 20);
    }

    public void setMM(double height) {
        determineSetpoint();
        elevatorDriverMainR1.set(ControlMode.MotionMagic, feetToPulses(commandedHeight),
        DemandType.ArbitraryFeedForward, KStatic);
    }

    public void stop(){
    elevatorDriverMainR1.set(ControlMode.PercentOutput, 0);
    }

    public void performMainProcessing() {
        configureLiftUpPID();
        determineSetpoint();
        setMM(commandedHeight);
    }

}
