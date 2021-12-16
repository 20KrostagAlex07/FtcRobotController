package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="singleDegreeArmHoldtoMoveJava", group="Pushbot")
public class PID extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    double errorPrior[] = new double[4]; //Uses 4 because that's how many different buttons use pidControlArm();
    double integralPrior[] = new double[4];
    double lastLoopTime[] = new double[4];
    int correctCount[] = new int[4];

    int pidCount = 0;
    int notPressedCount = 0;

    private AnalogInput potentiometer = null;
    private DcMotor arm;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor duckies;
    private Servo grabber;
    private Servo wrist;

    @Override
    public void runOpMode() {
        potentiometer = hardwareMap.analogInput.get("potentiometer");
        arm = hardwareMap.dcMotor.get("arm1");
        frontLeft = hardwareMap.dcMotor.get("front_left");
        frontRight = hardwareMap.dcMotor.get("front_right");
        backLeft = hardwareMap.dcMotor.get("back_left");
        backRight = hardwareMap.dcMotor.get("back_right");
        duckies = hardwareMap.dcMotor.get("duckies");
        grabber = hardwareMap.servo.get("grabber");
        wrist = hardwareMap.servo.get("wrist");


        while(!opModeIsActive()){}

        while(opModeIsActive()){

            pidControlArm(0.3, true, 0.5, pidCount);

            if(notPressedCount == pidCount){
                arm.setPower(0);
            }

            pidCount = 0;
            notPressedCount = 0;
            telemetry.addData(">", potentiometer.getVoltage());
            telemetry.update();
        }
    }
    public void pidControlArm(double desiredPosition, boolean control, double powerCap, int arrayPos){
        pidCount += 1;
        if(control){
            double changeInTime = (runtime.seconds() - lastLoopTime[arrayPos]);
            double error = 0;
            double integral = 0;
            double derivative = 0;
            double output = 0;
            double kp = 1;
            double ki = 3.4;
            double kd = .04;
            error = -(desiredPosition - potentiometer.getVoltage());
            if(Math.abs(error) > 0.1){
                integralPrior[arrayPos] = 0;
            }
            integral = integralPrior[arrayPos] + error*changeInTime;
            derivative = (error-errorPrior[arrayPos])/changeInTime;
            output = kp*error + ki*integral + kd*derivative;
            if(Math.abs(output) > powerCap){
                output = Math.signum(output)*powerCap;
            }
            telemetry.addData("p", kp*error);
            telemetry.addData("i", ki*integral);
            telemetry.addData("d", kd*derivative);
            errorPrior[arrayPos] = error;
            integralPrior[arrayPos] = integral;
            lastLoopTime[arrayPos] = runtime.seconds();
            if(Math.abs(error) <= 0.02){
                correctCount[arrayPos] += 1;
            }
            else{
                correctCount[arrayPos] = 0;
            }
            if(correctCount[arrayPos] <= 50){
                arm.setPower(output);
            }
            else{
                arm.setPower(0);
            }
        }
        else{
            notPressedCount += 1;
            lastLoopTime[arrayPos] = runtime.seconds();
        }
    }
}