package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI {

    public XboxController m_driveController = new XboxController(0);
    public XboxController m_mechController = new XboxController(1);

    

    public JoystickButton m_aButton = new JoystickButton(m_mechController, Constants.a_button);
    public JoystickButton m_bButton = new JoystickButton(m_mechController, Constants.b_button);
    public JoystickButton m_xButton = new JoystickButton(m_mechController, Constants.x_button);
    public JoystickButton m_yButton = new JoystickButton(m_mechController, Constants.y_button);
    public JoystickButton m_leftBumper = new JoystickButton(m_mechController, Constants.left_bumper);

    public Button m_rightTrigger = buttonFromAxis(m_mechController, Constants.right_trigger);
    public Button m_leftTrigger = buttonFromAxis(m_mechController, Constants.left_trigger);

    public JoystickButton d_aButton = new JoystickButton(m_driveController, Constants.a_button);
    

    private Button buttonFromAxis(XboxController controller, int axis) {
        return new Button() {
            @Override
            public boolean get() {
                return Math.abs(controller.getRawAxis(axis)) > 0.05;
            }
        };
    }
}
