/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import frc.utilities.Xbox;
import frc.utilities.Toggler;
import frc.systems.HatchMech;


public class ElevatorSmartMotion {

    CANSparkMax elevatorSpark;
	CANSparkMax elevatorSparkFollower;
    CANEncoder m_Encoder;
    
    public ElevatorSmartMotion(int elevator1CANPort, int elevator2CANPort) {
		elevatorSpark = new CANSparkMax(elevator1CANPort, MotorType.kBrushless);
		elevatorSparkFollower = new CANSparkMax(elevator2CANPort, MotorType.kBrushless);
		m_Encoder = elevatorSpark.getEncoder();

		elevatorSpark.restoreFactoryDefaults();
		elevatorSparkFollower.restoreFactoryDefaults();

		//elevatorSparkFollower.follow(elevatorSpark);

    }

    public void performMainProcessing(){
        if (Robot.xboxJoystick.getRawButton(Xbox.Y)) {
            elevatorSpark.set(0.4);
            //elevatorSparkFollower.set(0.4);
        }
    }

}
