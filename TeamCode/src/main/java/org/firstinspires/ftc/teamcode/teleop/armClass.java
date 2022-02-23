package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


public class armClass {

	/**
	 * construct PID controller 
	 * @param Kp Proportional coefficient  
	 * @param Ki Integral coefficient 
	 * @param Kd Derivative coefficient 
	 */
	public armPID(double Kp, double Ki, double Kd) {

	}

	/**
	 * update the PID controller output
	 * @param target where we would like to be, also called the reference
	 * @param state where we currently are, I.E. motor position 
	 * @return the command to our motor, I.E. motor power 
	 */
	public double armControl(double target, double state) {
		// PID logic and then return the output 
	}
}
