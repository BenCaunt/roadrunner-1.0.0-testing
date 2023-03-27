package org.firstinspires.ftc.teamcode.tuning;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class MotorTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {

		DcMotor leftFront = hardwareMap.get(DcMotorEx.class, "frontleft");
		DcMotor leftBack = hardwareMap.get(DcMotorEx.class, "backleft");
		DcMotor rightBack = hardwareMap.get(DcMotorEx.class, "backright");
		DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "frontright");

		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
		leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

		waitForStart();

		while (opModeIsActive()) {
			telemetry.addData("leftFront",leftFront.getCurrentPosition());
			telemetry.addData("leftBack",leftBack.getCurrentPosition());
			telemetry.addData("rightBack",rightBack.getCurrentPosition());
			telemetry.addData("rightFront",rightFront.getCurrentPosition());
			telemetry.update();
		}

	}
}
