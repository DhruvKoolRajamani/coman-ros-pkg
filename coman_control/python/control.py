#!/usr/bin/env python
""" Control node for a leg """

import rospy
from std_msgs.msg import Float64  # , String
from sensor_msgs.msg import JointState

import numpy as np

class EffortController(object):
    """ Open-loop position controller """

    def __init__(self, timestep):
        super(EffortController, self).__init__()
        self._timestep = timestep

        # Parameters
        namespace = rospy.get_param("namespace_1_1")
        control_prefix = namespace + "/control/config"
        self.n = rospy.get_param(
            namespace+"/n_joints"
        )
        self.freq = 1

        # Body
        self.pubs_body = [
            rospy.Publisher(
                (
                    control_prefix
                    + "/joint_effort_controller_base_body_{}/command".format(i)
                ),
                Float64,
                queue_size=10
            )
            for i in range(self.n)
        ]

        # Legs
        joint_control_name = (
            "/joint_effort_controller_body_leg_{}_{}_{}/command"
        )
        self.legs_joints_types = ["upp", "upp2", "inter", "low"]
        self.sides = ["L", "R"]
        self.pubs_legs = [
            [
                [
                    rospy.Publisher(
                        (
                            control_prefix
                            + joint_control_name.format(i, side, joint)
                        ),
                        Float64,
                        queue_size=10
                    )
                    for joint in self.legs_joints_types
                ] for side in self.sides
            ] for i in range(self.n)
        ]

        # Legs feedback
        self.joints_names = [
            [
                [
                    "body_leg_{}_{}_{}".format(i, side, joint)
                    for joint in self.legs_joints_types
                ] for side in self.sides
            ] for i in range(self.n)
        ]
        self.joints = np.zeros([self.n, 2, len(self.legs_joints_types)])
        rospy.Subscriber(
            namespace + "/joint_states",
            JointState,
            self.joints_positions
        )

        # One CPG (Multiple CPGs should be used in the future)
        self.cpg = CPG(
            omega=2*np.pi*self.freq,
            weight=0.0,
            phase_desired=0.0,
            timestep=self._timestep
        )

        # Control timer
        rospy.Timer(rospy.Duration(self._timestep), self.control)
        return

    def joints_positions(self, data):
        """ Get joints positions """
        for body_i, names_body in enumerate(self.joints_names):
            for side_i, names_side in enumerate(names_body):
                for joint_i, name_joint in enumerate(names_side):
                    self.joints[body_i][side_i][joint_i] = (
                        data.position[data.name.index(name_joint)]
                    )
        return

    def control(self, event=None, verbose=False):
        if verbose:
            print(event)
        self.body_control()
        self.legs_control()
        return

    def body_control(self):
        """ Body (spine) control """
        t = rospy.get_time()
        for i, pub in enumerate(self.pubs_body):
            cmd = (
                0.0*np.sin(2*np.pi*self.freq*t+i/self.n*2*np.pi) +
                0  # 0.1*np.sin(i/n*2*np.pi)
            )
            cmd = 0
            pub.publish(cmd)
        return

    def legs_control(self):
        """ Legs control """
        # t = rospy.get_time()
        pi2 = np.pi/2
        # pi4 = np.pi/4
        pi8 = np.pi/8
        pi16 = np.pi/16
        # w = 2.*np.pi*self.freq
        x = 2.
        cpg_phase = self.cpg.integrate()  # w*t
        for i in range(self.n):
            for side_i, side in enumerate(self.sides):
                for leg_i, joint_type in enumerate(self.legs_joints_types):
                    phase_desired = x*i*2*np.pi/self.n + side_i*np.pi
                    cmd = (
                        pi16 + pi16*np.sin(cpg_phase + phase_desired)
                        if joint_type == "upp"
                        else pi8*np.sin(cpg_phase + phase_desired + pi2)
                        if joint_type == "upp2"
                        else pi8 + pi16*np.sin(cpg_phase + phase_desired)
                        if joint_type == "inter"
                        else 0
                    )
                    self.pubs_legs[i][side_i][leg_i].publish(cmd)
        return


def main():
    """ Main """
    rospy.init_node('body_control', anonymous=False)
    timestep = 0.01  # seconds
    controller = PositionController(timestep)
    rospy.spin()
    del controller
    return


def test():
    """ Test CPG """
    dt = 1e-3
    cpg = PositionCPG(
        offset=1,
        amplitude=1,
        omega=2*np.pi*1,
        weight=10,
        phase_desired=0,
        timestep=dt
    )
    duration = 1.
    n = int(duration/dt)
    thetas = np.zeros(n)
    thetas[0] = np.real(cpg.theta)
    off = 1
    phase_prev = np.linspace(0+off, 2*np.pi+off, n)
    for i, _ in enumerate(thetas[:-1]):
        phase_diff = (
            ((np.real(cpg.theta) - phase_prev[i] + np.pi) % (2*np.pi)) - np.pi
        )
        thetas[i+1] = np.real(cpg.integrate(phase_diff))
    print("thetas:\n{}".format(thetas))
    import matplotlib.pyplot as plt
    # Phase plots
    plt.figure("Phases")
    plt.plot([i*dt for i in range(n)], thetas, label=r"$\theta_i$")
    plt.plot([i*dt for i in range(n)], phase_prev, label=r"$\theta_{i-1}$")
    plt.xlabel("Time")
    plt.ylabel("Phase")
    plt.legend(loc="best")
    plt.grid(True)
    # Phase difference plot
    plt.figure("Phases difference")
    plt.plot([i*dt for i in range(n)], thetas - phase_prev)
    plt.xlabel("Time")
    plt.ylabel("Phase")
    plt.grid(True)
    plt.show()
    return


if __name__ == "__main__":
    # test()
    try:
        main()
    except rospy.ROSInterruptException:
        pass
