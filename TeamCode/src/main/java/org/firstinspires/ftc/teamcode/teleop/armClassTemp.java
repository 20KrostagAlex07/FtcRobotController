//No errors except for the before-stated argument issues with "ArmPID"

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import FreightFrenzy.TeleOp.ArmTest;

public class ArmClass {
	
	double kP;
	double kI;
	double kD;

	public double ArmPID(double kP, double kI, double kD) {
	return 0;
	}

	
	 //update the PID controller output
	 // @return the command to our motor, I.E. motor power 
	 
	public double ArmControl(double target, double state) {
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
		 	return 0;
	}
}
