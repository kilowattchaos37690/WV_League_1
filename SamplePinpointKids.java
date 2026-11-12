package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "SamplePinpointKids (Blocks to Java)")
public class SamplePinpointKids extends LinearOpMode {

  @Override
  public void runOpMode() {
    RobotMyBlocks.bindFromLinearOpMode(this);
    RobotPinpointMyBlocks.bindFromLinearOpMode(this);

    RobotMyBlocks.initRobot();
    RobotPinpointMyBlocks.initPinpoint();
    RobotPinpointMyBlocks.setPose(0, 0, 0);
    telemetry.setMsTransmissionInterval(100);
    telemetry.addData("Status", "Ready — Sample Pinpoint (Kids)");
    telemetry.addData("Tip", "turn 90 CW (pose PIDF)");
    telemetry.update();
    waitForStart();
    if (!opModeIsActive()) {
      return;
    }
    RobotPinpointMyBlocks.turnDegreesOdo(90, 0.15);
    RobotPinpointMyBlocks.reportPose();
    telemetry.update();
    sleep(5000);
    RobotPinpointMyBlocks.stopDriveOdo();
    RobotMyBlocks.stopDrive();
    telemetry.addData("Status", "Auto done");
    telemetry.update();
    sleep(500);
  }
}
