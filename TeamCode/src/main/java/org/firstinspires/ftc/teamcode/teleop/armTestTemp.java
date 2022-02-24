//CURRENT ERRORS

//1. Stop() is not working--possible fix by using regular "extends OpMode"
//2. ArmClass ArmPID =  new ArmClass (0.7, 0.0, 0.0) does not work--it says that arguments arent required but i have no idea why!
//3. thats all for now

package FreightFrenzy.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.ArmClass;


@TeleOp(name = "Arm Test", group = "The Real Deal")

public class ArmTest extends LinearOpMode {

	 // Declare OpMode members.
	private ElapsedTime runtime = new ElapsedTime();
	private DcMotor arm;

	private int dir = -1;
	private double previousHeading = 0;
	private double integratedHeading = 0;
	ArmClass ArmPID = new ArmClass(0.7, 0.0, 0.0);
	/*
	 * Code to run ONCE when the driver hits INIT
	 */
	@Override
	public void runOpMode() {
		telemetry.addData("Status", "Initialized");

		// Initialize the hardware variables. Note that the strings used here as parameters
		// to 'get' must correspond to the names assigned during the robot configuration
		// step (using the FTC Robot Controller app on the phone).

		arm = hardwareMap.get(DcMotor.class, "arm1");
	


		// Tell the driver that initialization is complete.
		telemetry.addData("Status", "Initialized");

		//reset timer
		runtime.reset();
		
		arm.setDirection(DcMotorSimple.Direction.REVERSE);

		arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		// disables the default velocity control
		// this does NOT disable the encoder from counting, 
		// but lets us simply send raw motor power.
		arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	
		waitForStart();
	
	while (opModeIsActive()) {
		
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
	
	//PID logic
	double update = ArmPID.ArmControl(targetPosition, arm.getCurrentPosition());
	//assign arm the PID update value 
	arm.setPower(update);
	
	if(gamepad2.b){
		break;
	}
	}
	}
	}
	
	
	}

	}

	/*
	 * Code to run ONCE after the driver hits STOP
	 */
