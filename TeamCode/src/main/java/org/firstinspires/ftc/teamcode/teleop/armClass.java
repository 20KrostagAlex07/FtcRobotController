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
	 // @return the command to our motor, I.E. motor power 
	 
	public void armControl(double target, double state) {
		// PID logic and then return the output 
		
		 //initialize stuff
		 double error = target - state;
		 double errorLast = 0;
		 ElapsedTime timer = new ElapsedTime();
		 
		 //PID logic
		 error = target - state;
		 
		 double outP = kP * error;
		 double outI = 0;
		 double outD = kD * ((error - errorLast) / timer.seconds());
		 double update = outP + outI + outD; 
			 
		 //reset stuff
		 errorLast = error;
		 timer.reset();
	}
}
		
		 //initialize stuff
		 double errorLast;
		 elapsed timer = new ElapsedTime();
		 
		 while(error >= 10){
		 
		 //PID logic
		 double error = target - state;
		 double outP = kP * error;
		 double outI = 0;
		 double outD = kD * ((error - errorLast) / timer.seconds());
		 double update = outP + outI + outD; 
			 
		 //reset stuff
		 double errorLast = error;
		 timer.reset();
		 }
		 
	}
}
