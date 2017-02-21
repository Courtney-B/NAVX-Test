
package org.usfirst.frc.team2723.robot;

import edu.wpi.first.wpilibj.IterativeRobot;


import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.cscore.UsbCamera;


import org.usfirst.frc.team2723.robot.commands.ExampleCommand;
import org.usfirst.frc.team2723.robot.subsystems.ExampleSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

/**
 * This is a demo program showing the use of the navX MXP to implement
 * the "rotate to angle", "zero yaw" and "drive straight" on a Tank
 * drive system.
 *
 * If Left Joystick Button 0 is pressed, a "turn" PID controller will 
 * set to point to a target angle, and while the button is held the drive
 * system will rotate to that angle (NOTE:  tank drive systems cannot simultaneously
 * move forward/reverse while rotating).
 *
 * This example also includes a feature allowing the driver to "reset"
 * the "yaw" angle.  When the reset occurs, the new gyro angle will be
 * 0 degrees.  This can be useful in cases when the gyro drifts, which
 * doesn't typically happen during a FRC match, but can occur during
 * long practice sessions.
 *
 * Finally, if Left Joystick button 2 is held, the "turn" PID controller will
 * be set to point to the current heading, and while the button is held,
 * the driver system will continue to point in the direction.  The robot 
 * can drive forward and backward (the magnitude of motion is the average
 * of the Y axis values on the left and right joysticks).
 *
 * Note that the PID Controller coefficients defined below will need to
 * be tuned for your drive system.
 */
public class Robot extends IterativeRobot implements PIDOutput {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();
	
	//Defines the variables as members of our Robot class
    CANTalon frontLeftMotor = new CANTalon(4);
    CANTalon rearLeftMotor = new CANTalon(3);
    CANTalon frontRightMotor = new CANTalon(2);
    CANTalon rearRightMotor = new CANTalon(1);
    CANTalon lift = new CANTalon(5);
    CANTalon intake = new CANTalon(6);
    CANTalon wow = new CANTalon(7);
    CANTalon lShooter = new CANTalon(8);
    CANTalon rShooter = new CANTalon (9);
    
    RobotDrive myRobot;  // class that handles basic drive operations
    Joystick joystick;  // set to ID 1 in DriverStation
    Joystick joystick2;
    AHRS ahrs;
    
    PIDController turnController;
    double rotateToAngleRate;
    double leftRobotSpeed = 0.4;
    double rightRobotSpeed = 0.4;
    double magnitude2;
    
    /* The following PID Controller coefficients will need to be tuned */
    /* to match the dynamics of your drive system.  Note that the      */
    /* SmartDashboard in Test mode has support for helping you tune    */
    /* controllers by displaying a form where you can enter new P, I,  */
    /* and D constants and test the mechanism.                         */
    
    static final double kP = 0.02;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    
    static final double kToleranceDegrees = 2.0f;    
    
    static final double kTargetAngleDegrees = 180.0f;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture();
		//UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture();
		
		//SWITCHING BETWEEN CAMERAS TEST
		/*try {
			//DUAL CAMERA CODE
			if (joystick2.getRawButton(4))
			{
				camera2 = CameraServer.getInstance().startAutomaticCapture();
			}
			else
			{
				camera1 = CameraServer.getInstance().startAutomaticCapture();
			}
		}
		catch (NullPointerException ex)
		{
			DriverStation.reportError("Error instantiating camera:  " + ex.getMessage(), true);
		} */

		myRobot = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
	       //myRobot.setExpiration(0.01);
	        myRobot.setSafetyEnabled(false);
	        joystick = new Joystick(0);
	        joystick2 = new Joystick(1);
	        
	        frontLeftMotor.enableControl();
	        rearLeftMotor.enableControl();
	        frontRightMotor.enableControl();
	        rearRightMotor.enableControl();
	        lift.enableControl();
	        intake.enableControl();
	        wow.enableControl();
	        lShooter.enableControl();
	        rShooter.enableControl();
	        
	        try {
				/***********************************************************************
				 * navX-MXP:
				 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
				 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
				 * 
				 * navX-Micro:
				 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
				 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
				 * 
				 * Multiple navX-model devices on a single robot are supported.
				 ************************************************************************/
	            ahrs = new AHRS(SPI.Port.kMXP); 
	        } catch (RuntimeException ex ) {
	            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
	        }
	        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
	        turnController.setInputRange(-180.0f,  180.0f);
	        turnController.setOutputRange(-0.8, 0.8);
	        turnController.setAbsoluteTolerance(kToleranceDegrees);
	        turnController.setContinuous(true);
	        turnController.disable();
	        
	        /* Add the PID Controller to the Test-mode dashboard, allowing manual  */
	        /* tuning of the Turn Controller's P, I and D coefficients.            */
	        /* Typically, only the P value needs to be modified.                   */
	        LiveWindow.addActuator("DriveSystem", "RotateController", turnController);   
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();
		double magnitude;
		boolean rotateToAngle2;
		ahrs.zeroYaw();
		turnController.disable();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null) autonomousCommand.start();
		
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
		
		if (!turnController.isEnabled()) {
			turnController.setSetpoint(90.0f);
			rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
			turnController.enable();
		}
		double leftStickValue = rotateToAngleRate;
		double rightStickValue = -rotateToAngleRate;
		myRobot.tankDrive(leftStickValue,  rightStickValue);
		
		
		
/*		if(!turnController.isEnabled()) {
			// Acquire current yaw angle, using this as the target angle.
			turnController.setSetpoint(90.0f);
			rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
			turnController.enable();
		}
		double magnitude = 0.2;
		double leftStickValue = magnitude + rotateToAngleRate;
		double rightStickValue = magnitude - rotateToAngleRate;
		myRobot.tankDrive(leftStickValue,  rightStickValue);
		*/
		//rotateToAngleRate = 0;
		//System.out.println("Test 0!");
				/*
				if(!turnController.isEnabled()) {
					// Acquire current yaw angle, using this as the target angle.
					turnController.setSetpoint(ahrs.getYaw());
					rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
					turnController.enable();
					
				magnitude2 = (rightRobotSpeed + leftRobotSpeed);
				double leftStickValue = magnitude2 + rotateToAngleRate;
				double rightStickValue = magnitude2 - rotateToAngleRate;
				myRobot.tankDrive(leftStickValue,  rightStickValue);
				Timer.delay(0.5);
				System.out.println("Test 1!");
				
				//turn around 180.0f
				
				//rotateToAngleRate = 180;
				
				
				 if (!turnController.isEnabled()) {
					//System.out.println("Test 2!");
					
					turnController.setSetpoint(179.9f);
					rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
					turnController.enable();
				}
				
				//turnController.setSetpoint(179.9f);
				double leftStickValue1 = rotateToAngleRate;
				double rightStickValue1 = rotateToAngleRate;
				//myRobot.tankDrive(leftStickValue1,  rightStickValue1); 
				
				//and drive back to starting position
				//rotateToAngleRate = 0;
				//System.out.println("Test 3!");
				myRobot.tankDrive(leftStickValue,  rightStickValue);
				
				Timer.delay(2);
				myRobot.tankDrive(0,0);}
				*/
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		while (isOperatorControl() && isEnabled()) {
			
			double slowdown = joystick.getRawAxis(2); //3=right trigger 2= left trigger
			double left = -joystick.getRawAxis(1); // 1=left stick y
	    	double right = -joystick.getRawAxis(5); //5-left stick y
	    	if (slowdown > 0.5) {
	    		left = left/3;
	    		right = right/3;
	    	}
	    	
	    	 //BUTTON CODE
            if(joystick.getRawButton(5)) // Left Bumper Button on the Logitech F310 joystick
            {
            	intake.set(1); 
            } else if(joystick.getRawButton(6)) // Right Bumper Button on joystick
            {
            	intake.set(-1);
            } else {
            	intake.set(0);
            }
	    	
			myRobot.tankDrive(left, right);
			//Controller 2 Code
			lift.set(joystick2.getRawAxis(1));
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		rotateToAngleRate = output;
	}
}
