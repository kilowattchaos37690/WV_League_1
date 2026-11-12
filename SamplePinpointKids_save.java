package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Kid-facing Pinpoint Auto — CONVERTIBLE to Blocks.
 *
 * Prefer lower max power (0.35–0.45) while tuning smoothness.
 * RobotMyBlocks.bind / RobotPinpointMyBlocks.bind required for Java OpModes.
 */
@Autonomous(name = "Sample Pinpoint (Kids)", group = "Sample")
public class SamplePinpointKids extends LinearOpMode {

  @Override
  public void runOpMode() {
    RobotMyBlocks.bindFromLinearOpMode(this);
    RobotPinpointMyBlocks.bindFromLinearOpMode(this);

    RobotMyBlocks.initRobot();
    RobotPinpointMyBlocks.initPinpoint();
    RobotPinpointMyBlocks.setPose(0.0, 0.0, 0.0);

    telemetry.addData("Status", "Ready — Sample Pinpoint (Kids)");
    telemetry.addData("Tip", "fwd → strafe → back (softer pose PIDF)");
    telemetry.update();
    waitForStart();

    if (!opModeIsActive()) {
      return;
    }

    RobotPinpointMyBlocks.driveForwardOdo(10.0, 0.4);
    RobotPinpointMyBlocks.strafeRightOdo(6.0, 0.35);
    RobotPinpointMyBlocks.driveForwardOdo(-10.0, 0.4);
    RobotPinpointMyBlocks.reportPose();
    RobotPinpointMyBlocks.stopDriveOdo();
    RobotMyBlocks.stopDrive();

    telemetry.addData("Status", "Auto done");
    telemetry.update();
    sleep(500);
  }
}
