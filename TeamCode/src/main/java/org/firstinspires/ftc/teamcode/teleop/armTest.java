//CURRENT ERRORS

//1. Stop() is not working--possible fix by using regular "extends OpMode"
//2. ArmClass ArmPID =  new ArmClass (0.7, 0.0, 0.0) does not work--it says that arguments arent required but i have no idea why!
//3. thats all for now

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

import https/github.com/JavaJokers/FtcRobotController/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/armClass.java


//Start TeleOp
@TeleOp(name = "Real Arm Test", group = "The Real Deal")

public class armTest extends OpMode {

	 // Declare OpMode members.
	private ElapsedTime runtime = new ElapsedTime();
	private DcMotor arm;

	private int dir = -1;
	private double previousHeading = 0;
	private double integratedHeading = 0;
	armClass armPID = new armClass(0.7, 0.0, 0.0);
	
	
	@Override
	public void init() {
		telemetry.addData("Status", "Initialized");

		// Initialize the hardware variables. Note that the strings used here as parameters
		// to 'get' must correspond to the names assigned during the robot configuration
		// step (using the FTC Robot Controller app on the phone).

		arm = hardwareMap.get(DcMotor.class, "arm1");
	


		// Tell the driver that initialization is complete.
		telemetry.addData("Status", "Initialized");
	}
		
	
	@Override
	public void start() {

		//reset timer
		runtime.reset();
		
		arm.setDirection(DcMotorSimple.Direction.REVERSE);

		arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		// disables the default velocity control
		// this does NOT disable the encoder from counting, 
		// but lets us simply send raw motor power.
		arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	
	}
	
		
	@Override
	public void loop() {
//GamePad 2 Controls
		
		
		//set arm power
		//if statement is the arm's counter-force against gravity
		//else if statement is the arm's slow mode
		//else statement is regular arm speed
		if (gamepad2.right_bumper) {
			arm.setPower(-0.1);
		} else if (gamepad2.left_bumper) {
			arm.setPower(gamepad2.left_stick_y * 0.35);
		} else {
			arm.setPower(gamepad2.left_stick_y * 0.7);
		}

	if(gamepad2.a){
	double targetPosition = 360;
	
	double error = targetPosition - arm.getCurrentPosition();
	
	while (error >= 100){
	
		//Use PID logic
		double update = armPID.armControl(targetPosition, arm.getCurrentPosition());
		
		//Assign arm the PID update value 
		arm.setPower(update);
		
		//Emergency stop
		if(gamepad2.b){
			break;
			} //gamepad2.b end
		
		} //while statement end
	} //gamepad1.a end
	
} //loop() end
	
	
	@Override
	public void stop() {
	//Code to run ONCE after the driver hits STOP
	} //stop() end
} //class end bracket
