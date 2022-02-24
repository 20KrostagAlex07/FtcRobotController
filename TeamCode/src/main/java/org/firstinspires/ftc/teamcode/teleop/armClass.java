//No errors except for the before-stated argument issues with "armPID"

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


//make class for PID
public class armClass {
	
	
	/**
	* @param kP - proportional value
	* @param kI - integral value
	* @param kD - deriative value
	*/
	
	public void armPID(double kP, double kI, double kD) {
			
	}

	
	 //update the PID controller output
	 
	public double armControl(double target, double state, double error, double errorLast, double time) {
		// PID logic and then return the output 
		 
		 //PID logic
		 
		 double outP = kP * error;
		 double outI = 0;
		 double outD = kD * ((error - errorLast) / time);
		 double output = outP + outI + outD;
		 errorLast = error;

		 //return output value to motor
		 return output;
		 return errorLast
	}
}
		
}
