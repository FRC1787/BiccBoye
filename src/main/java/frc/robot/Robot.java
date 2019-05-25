/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PIDBase;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDInterface;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

import java.lang.Math;



public class Robot extends TimedRobot {


/*************************/
/* OBJECT INITIALIZATION */
/*************************/


private final Articulation articulation = Articulation.getInstance();

//Drive Motor Controller Objects
public WPI_TalonSRX FrontRight = new WPI_TalonSRX(4);
public WPI_TalonSRX FrontLeft = new WPI_TalonSRX(2);
public WPI_TalonSRX BackRight = new WPI_TalonSRX(3);
public WPI_TalonSRX BackLeft = new WPI_TalonSRX(4);

//Articulation Motor Controller Objects
public WPI_TalonSRX AFrontRight = new WPI_TalonSRX(6);
//public WPI_TalonSRX AFrontLeft = new WPI_TalonSRX();
//public WPI_TalonSRX ABackRight = new WPI_TalonSRX();
//public WPI_TalonSRX ABackLeft = new WPI_TalonSRX(8);

//PID Test Talons
public TalonSRX TalonSrx = new TalonSRX(15);
public WPI_TalonSRX WPITalon = new WPI_TalonSRX(16);

//Joystick Objects
public Joystick RightStick = new Joystick(0);
public Joystick LeftStick = new Joystick(1);

//Encoder Object Setup
public static Encoder EFrontRight = new Encoder(0, 1);       //Encoder 1 Setup
//public DigitalInput Encoder1A = new DigitalInput(0);
//public DigitalInput Encoder1B = new DigitalInput(1);
public Encoder EFrontLeft = new Encoder(2,3);         //Encoder 2 Setup
//public DigitalInput Encoder2A = new DigitalInput(2);
//public DigitalInput Encoder2B = new DigitalInput(3);



/*************/
/* VARIABLES */
/*************/


//final PID values
private final double PROPORTIONAL_TWEAK_CONSTANT = 0.0004; //0.0004
private final double INTEGRAL_TWEAK_CONSTANT = 0.0000072; //.000007
private final double DERIVATIVE__TWEAK_CONSTANT = 0;
private final double ACCEPTABLE_ERROR_RANGE = 0.0;

//PID variables 
private double error = 0;
private double proportional = 0;
private double derivative = 0;
private double integral = 0; 
private double previousError = 0;
private double pIDMotorVoltage = 0;

//Things to use
public double Encoder1 = EFrontRight.get();


  public void robotInit() 
  {
    this.setDashboard();
  }

  public void robotPeriodic() 
  {
    this.setDashboard();
    this.getDashboard();
  }

  public void autonomousInit()  {}

  public void autonomousPeriodic() {}

  public void teleopPeriodic() 
  {
    this.myPeriodic();
  }

  public void testPeriodic() {}

  public void myPeriodic() 
  {
    
    
    
    //Drive Wheel Logic
    if (Math.abs(RightStick.getX()) >= Math.abs(RightStick.getY()) && Math.abs(RightStick.getX()) >= Math.abs(RightStick.getZ())) 
    {
      FrontRight.set(RightStick.getX());
    }
    
    if (Math.abs(RightStick.getY()) >= Math.abs(RightStick.getX()) && Math.abs(RightStick.getY()) >= Math.abs(RightStick.getZ())) 
    {
      FrontRight.set(RightStick.getY());
    }

    if (Math.abs(RightStick.getZ()) >= Math.abs(RightStick.getX()) && Math.abs(RightStick.getZ()) >= Math.abs(RightStick.getY())) 
    {
      FrontRight.set(RightStick.getZ());
    }


    //Module Articulation Logic

    /* For future reference I want to make it so a degree value coming from the joystick
    is translated to be representative of a value from the encoder so that when an angle is recieved from the joystick
    the robot tries to set the encoder value equal to the joystick value by articulating the motor */

    //I freaking did it

    EFrontRight.setReverseDirection(false);

    if (RightStick.getThrottle() > 0)
    {
      pIDDrive(AFrontRight, angull());
    }
    else
    {
      AFrontRight.set(0);
    }


    if (RightStick.getRawButton(2)) //Talon build in PID????
    {
      //AFrontRight.config_kP(0, 0);
      //AFrontRight.config_kI(0, 0);
      //AFrontRight.config_kD(0, 0);
      //AFrontRight.configMotionCruiseVelocity(500);
      //AFrontRight.Motion
    }
  
  }

  


  /* Sets a motor to seek a specific orientation when EncoderNum 
  is the value of the encoder and JoyNum is the angle being read from
  the joystick */


  /********************/
  /*FANCY MATH METHODS*/
  /********************/

  public double fixInput(double joyNum)
  {
    if (joyNum > .15 || joyNum < .15)
    return joyNum;
    else
    return 0;
  }

  public double angull()
  {
    if (fixInput(RightStick.getX()) == 0 && fixInput(RightStick.getY()) == 0)
    return 0;
    else
    return RightStick.getDirectionDegrees()*5.555;
  }

  public double raydeeins()
  {
    if (fixInput(RightStick.getX()) == 0 && fixInput(RightStick.getY()) == 0)
    return 0;
    else
    return RightStick.getDirectionRadians();
  }
 
  private double truncateMotorOutput(double motorOutput) 
  {
		if (motorOutput > 1) {
			return 0.5;
		} else if (motorOutput < -1) {
			return -0.5;
		} else {
			return motorOutput;
		}
  }
  
  public void pIDDrive(WPI_TalonSRX motor, double targetDiatance) // Enter target encoder value
	{ 

		error = targetDiatance - (EFrontRight.get());
		proportional = error;
		derivative = (previousError - error)/ 0.02;
		integral += previousError;
    previousError = error;
    //System.out.println(error);
		

    if (error > ACCEPTABLE_ERROR_RANGE || error < -ACCEPTABLE_ERROR_RANGE)
		{
			pIDMotorVoltage = truncateMotorOutput((PROPORTIONAL_TWEAK_CONSTANT * proportional) + (DERIVATIVE__TWEAK_CONSTANT * derivative) + (INTEGRAL_TWEAK_CONSTANT * integral));
      motor.set(pIDMotorVoltage);
      
    }
    else
		{
			proportional = 0;
			integral = 0;
			derivative = 0;
			previousError = 0;
      motor.set(0);
      //System.out.println(error);
    }
		
  }
  
  /***********/
  /*DASHBOARD*/
  /***********/
  public void setDashboard()
  {
    SmartDashboard.putNumber("Joystick Angle", RightStick.getDirectionDegrees());
    SmartDashboard.putNumber("Encoder Value", EFrontRight.get());
    SmartDashboard.putNumber("Angull", angull());
    SmartDashboard.putNumber("Raydeeins", raydeeins());
  }

  public void getDashboard(){}
  

}
 
  




// Stuff to remember
// rotating one of the encoders with the hole 90 degrees is 500 ticks
