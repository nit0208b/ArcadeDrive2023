package frc.robot.subsystems;

import edu.wpi.first.hal.ControlWord;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;


public class DriveSubsystem extends SubsystemBase {
	private static DriveSubsystem s_subsystem;

	WPI_TalonSRX m_left = new WPI_TalonSRX(4);
	WPI_TalonSRX m_right = new WPI_TalonSRX(5);

	ControlWord m_controlWord = new ControlWord();
	/** Prevents the motors from continuously being set to brake or coast mode */
	private boolean wasDisabled;

	public DriveSubsystem() {
		// Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("Drive subsystem already initialized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;
	}

	public static DriveSubsystem get() {
		return s_subsystem;
	}

	public void periodic() {
		// wasDisabled exists so the motors aren't constantly set to brake or coast mode
		// Without it, the code would continuously set the motors in brake or coast mode
		// If the robot is enabled, put the motors in brake mode
		if (isEnabledFast() && wasDisabled) {
			wasDisabled = false;
			// If the robot is disabled, put the motors in coast mode
		} else if (!isEnabledFast() && !wasDisabled) {
			wasDisabled = true;
		}
	}

	public void arcadeDrive(double straight, double left, double right) {
		tankDrive(DriveConstants.kSpeedLimitFactor * (straight - left + right),
				DriveConstants.kSpeedLimitFactor * (straight + left - right));
	}

	public void arcadeDrive(double straight, double turn) {
		tankDrive(DriveConstants.kSpeedLimitFactor * (straight - turn),
				DriveConstants.kSpeedLimitFactor * (straight + turn));
	}

	/**
	 * @param leftSpeed  Left motors percent output
	 * @param rightSpeed Right motors percent output
	 */
	public void tankDrive(double leftSpeed, double rightSpeed) {
		// TODO only set front? back should follow
		m_left.set(leftSpeed);
		m_right.set(-rightSpeed);
	}

	/**
	 * Hacky method that returns if the robot is enabled to bypass a WPILib bug with
	 * loop overruns
	 * 
	 * @author Jonathan Waters
	 * @return Whether or not the robot is enabled
	 */
	public boolean isEnabledFast() {
		DriverStationJNI.getControlWord(m_controlWord);

		return m_controlWord.getEnabled();
	}
}
