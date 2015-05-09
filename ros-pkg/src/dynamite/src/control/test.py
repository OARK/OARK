import rospy
import manager_proxy
import pos_controller
import torque_controller

def setup_test_pos():
    rospy.init_node('test_pos_node')
    man = manager_proxy.ManagerProxy('dxl_manager')
    fl = pos_controller.PosController(man, 'pos_fl', 'pi_out_port')
    fr = pos_controller.PosController(man, 'pos_fr', 'pi_out_port')
    return (fl,fr)

def setup_test_torque():
    rospy.init_node('test_torque_node')
    man = manager_proxy.ManagerProxy('dxl_manager')
    fl = torque_controller.TorqueController(man, 'wheel_fl', 'pi_out_port')
    fr = torque_controller.TorqueController(man, 'wheel_fr', 'pi_out_port')
    return (fl,fr)


def setup_car():
    rospy.init_node('car_node')
    man = manager_proxy.ManagerProxy('dxl_manager')
    fl = torque_controller.TorqueController(man, 'wheel_fl', 'pi_out_port')
    fr = torque_controller.TorqueController(man, 'wheel_fr', 'pi_out_port')
    bl = torque_controller.TorqueController(man, 'wheel_bl', 'pi_out_port')
    br = torque_controller.TorqueController(man, 'wheel_br', 'pi_out_port')

    arm_base = pos_controller.PosController(man, 'arm_base', 'pi_out_port')
    arm_wrist = pos_controller.PosController(man, 'arm_wrist', 'pi_out_port')
    arm_hand = pos_controller.PosController(man, 'arm_hand', 'pi_out_port')

    return (fl,fr,bl,br,arm_base,arm_wrist,arm_hand)


