// Dual camera switching example. 
// 1. Open SmartDashboard  
// 2. click View/Add/CameraServer Stream Viewer
// 3. click View/editable
// 4. right-click the camera field
// 5. in the properties tab, there is a feild called "Selected Camera Path". input "CameraSelection".
// 6. while in teleop, the "A" and "B" buttons control which camera is selected. 

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
  private final Joystick joy1 = new Joystick(0);
  private final JoystickButton frontSelect = new JoystickButton(joy1, 1); // A
  private final JoystickButton backSelect = new JoystickButton(joy1, 2); // B

  UsbCamera camera1;
  UsbCamera camera2;
  NetworkTableEntry cameraSelection;

@Override
public void robotInit() {
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);

    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
}

@Override
public void teleopPeriodic() {
    if (frontSelect.get()) {
        System.out.println("Setting front camera");
        cameraSelection.setString(camera2.getName());
    } else if (backSelect.get()) {
        System.out.println("Setting back camera");
        cameraSelection.setString(camera1.getName());
    }
}
}
