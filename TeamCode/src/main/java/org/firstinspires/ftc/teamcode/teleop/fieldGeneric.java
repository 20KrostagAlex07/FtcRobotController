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


@TeleOp(name = "Field Generic", group = "The Real Deal")

public class fieldGeneric extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm;
    private DcMotor duckies;
    private BNO055IMU imu;

    private Servo grabber;

    private float grabberPos = 160;

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    private boolean isA = false;
    private boolean wasA = false;
    private int i = 0;
    private int dir = -1;
    private double previousHeading = 0;
    private double integratedHeading = 0;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        arm = hardwareMap.get(DcMotor.class, "arm1");
        duckies = hardwareMap.get(DcMotor.class, "duckies");
        imu = hardwareMap.get(BNO055IMU.class, "gyro");
        grabber = hardwareMap.get(Servo.class, "grabber");

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        //Initialize gyro
        imu.initialize(parameters);

        //initiation grabber position
        grabberPos = 20;
        grabber.setPosition(grabberPos / 200);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //reset timer
        runtime.reset();

        //set directions
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);


        //set zeropowerbehavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double theta = gamepad1.right_stick_x * 1.1;
        double heading = -imu.getAngularOrientation().firstAngle;

        double x_rotated = x * Math.cos(heading) - y * Math.sin(heading);
        double y_rotated = x * Math.sin(heading) + y * Math.cos(heading);

        //this is a test

        double frontLeftPower = y_rotated + x_rotated + theta;
        double backLeftPower = y_rotated - x_rotated + theta;
        double frontRightPower = y_rotated - x_rotated - theta;
        double backRightPower = y_rotated + x_rotated - theta;

        // Put powers in the range of -1 to 1 only if they aren't already
        // Not checking would cause us to always drive at full speed
        if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
                Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(backRightPower), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        //change speed of robot
        if ((isA = gamepad1.a) && !wasA) {
            i++;
        }


        if (gamepad1.right_bumper) {
            frontLeft.setPower(frontLeftPower * 0.25 * dir);
            backLeft.setPower(backLeftPower * 0.25 * dir);
            frontRight.setPower(frontRightPower * 0.25 * dir);
            backRight.setPower(backRightPower * 0.25 * dir);
            telemetry.addLine("Speed one quarter");
        } else if (gamepad1.left_bumper) {
            frontLeft.setPower(frontLeftPower * 0.75 * dir);
            backLeft.setPower(backLeftPower * 0.75 * dir);
            frontRight.setPower(frontRightPower * 0.75 * dir);
            backRight.setPower(backRightPower * 0.75 * dir);
            telemetry.addLine("Speed 3/4");
        } else {
            frontLeft.setPower(frontLeftPower * dir);
            backLeft.setPower(backLeftPower * dir);
            frontRight.setPower(frontRightPower * dir);
            backRight.setPower(backRightPower * dir);
            telemetry.addLine("Speed full");
        }



        if(gamepad1.right_trigger == 1 && gamepad1.left_trigger == 1){
            imu.initialize(parameters);
        }


        //right stick grabber positions
        //also if this works well enough we could dedicate trigers to duckies and buttons to specific grabber positions
        double grabberPosStick = gamepad2.right_stick_x;
        if(gamepad2.right_stick_x){
            grabberPos = grabberPos + gamepad2.right_stick_x;
        }

        //triggers grabber positions
        if(gamepad2.left_trigger == 1){
            grabberPos = grabberPos + 4;
        } else if(gamepad2.right_trigger == 1){
            grabberPos = grabberPos - 4;
        }

        //buttons control grabber positions
        if(gamepad2.y) {
            grabberPos = 160;
        } else if(gamepad2.a) {
            grabberPos = 20;
        }


        //clamp grabberPos
        if (grabberPos > 160) {
            grabberPos = 160;
        } else if (grabberPos < 20) {
            grabberPos = 20;
        }

        //add emergency grabber opening
        if(gamepad2.dpad_down){
            grabber.setPosition(1);
            grabberPos = 160;
            grabber.setPosition(grabberPos / 200);

        }

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


        //set ducky motor
        if (gamepad1.x || gamepad2.x) {
            duckies.setPower(0.6);
        } else {
            duckies.setPower(0);
        }

        if(gamepad1.b || gamepad2.b){
            duckies.setPower(-0.7);
        } else {
            duckies.setPower(0);
        }

        //update grabber position
        grabber.setPosition(grabberPos / 200);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Heading", heading);
        telemetry.addData("Grabber Position", grabberPos);
        telemetry.update();

        wasA = isA;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        grabber.setPosition(160/200);
    }



}
