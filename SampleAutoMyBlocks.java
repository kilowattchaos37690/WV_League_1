package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Sample Autonomous using {@link RobotMyBlocks} (same routines kids get as Blocks).
 *
 * Blocks equivalent (after RobotMyBlocks is on the RC):
 *   initRobot
 *   driveForward 12 in @ 0.4
 *   turnDegrees 90 @ 0.35
 *   driveForward 6 in @ 0.4
 *   servosOpen
 *   stopDrive
 */
@Autonomous(name = "Sample Auto (MyBlocks)", group = "Sample")
public class SampleAutoMyBlocks extends LinearOpMode {

  @Override
  public void runOpMode() {
    RobotMyBlocks.bindFromLinearOpMode(this);
    RobotMyBlocks.initRobot();

    telemetry.addData("Status", "Ready — Sample Auto (MyBlocks)");
    telemetry.addData("Tip", "Tune COUNTS / PIDF in RobotMyBlocks.java");
    telemetry.update();
    waitForStart();

    if (!opModeIsActive()) {
      return;
    }

    // Same sequence kids would build from myBlocks
    RobotMyBlocks.driveForward(12.0, 0.4);
    RobotMyBlocks.turnDegrees(90.0, 0.35);
    RobotMyBlocks.driveForward(6.0, 0.4);
    RobotMyBlocks.servosOpen();
    RobotMyBlocks.stopDrive();

    telemetry.addData("Status", "Auto done");
    telemetry.update();
    sleep(500);
  }
}
