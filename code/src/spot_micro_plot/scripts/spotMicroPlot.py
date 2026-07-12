#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from spot_micro_kinematics_python.spot_micro_stick_figure import SpotMicroStickFigure
from spot_micro_kinematics_python.utilities import transformations
from math import pi
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation


class SpotMicroPlotNode(Node):
    def __init__(self):
        super().__init__('spot_micro_plot')

        self.latest_body_state = None

        self.create_subscription(Float32MultiArray, '/body_state', self._on_body_state, 10)

        self.fig = plt.figure()
        self.ax = p3.Axes3D(self.fig)
        self.ax.set_proj_type('ortho')
        self.ax.set_facecolor('black')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Z')
        self.ax.set_zlabel('Y')
        self.ax.set_xlim3d([-0.2, 0.2])
        self.ax.set_zlim3d([0, 0.3])
        self.ax.set_ylim3d([0.2, -0.2])

        self.sm = SpotMicroStickFigure(x=0, y=0.093, z=0)
        self.lines = self._create_initial_lines()

        self.lines_ani = animation.FuncAnimation(
            self.fig,
            self._update_lines,
            frames=1000,
            interval=100,
            blit=False,
        )

    def _on_body_state(self, msg):
        self.latest_body_state = msg

    def _create_initial_lines(self):
        coords = self.sm.get_leg_coordinates()
        lines = []

        for i in range(4):
            ind = -1 if i == 3 else i
            x_vals = [coords[ind][0][0], coords[ind + 1][0][0]]
            y_vals = [coords[ind][0][1], coords[ind + 1][0][1]]
            z_vals = [coords[ind][0][2], coords[ind + 1][0][2]]
            lines.append(self.ax.plot(x_vals, z_vals, y_vals, color='k')[0])

        plt_colors = ['r', 'c', 'b']
        for leg in coords:
            for i in range(3):
                x_vals = [leg[i][0], leg[i + 1][0]]
                y_vals = [leg[i][1], leg[i + 1][1]]
                z_vals = [leg[i][2], leg[i + 1][2]]
                lines.append(self.ax.plot(x_vals, z_vals, y_vals, color=plt_colors[i])[0])

        return lines

    def _update_lines(self, _frame):
        rclpy.spin_once(self, timeout_sec=0.0)
        if self.latest_body_state is None:
            return self.lines

        msg = self.latest_body_state
        foot_data = np.array([
            [msg.data[0], msg.data[1], msg.data[2]],
            [msg.data[3], msg.data[4], msg.data[5]],
            [msg.data[6], msg.data[7], msg.data[8]],
            [msg.data[9], msg.data[10], msg.data[11]],
        ])

        xpos = msg.data[12]
        ypos = msg.data[13]
        zpos = msg.data[14]

        phi = msg.data[15]
        theta = msg.data[16]
        psi = msg.data[17]

        self.sm.set_absolute_foot_coordinates(foot_data)
        temp_rot = transformations.rotxyz(phi, psi, theta)
        temp_pose = np.identity(4)
        temp_pose[0:3, 0:3] = temp_rot
        temp_pose[0, 3] = xpos
        temp_pose[1, 3] = ypos
        temp_pose[2, 3] = zpos
        self.sm.set_absolute_body_pose(temp_pose)

        coord_data = self.sm.get_leg_coordinates()

        line_to_leg_and_link_dict = {
            4: (0, 0), 5: (0, 1), 6: (0, 2),
            7: (1, 0), 8: (1, 1), 9: (1, 2),
            10: (2, 0), 11: (2, 1), 12: (2, 2),
            13: (3, 0), 14: (3, 1), 15: (3, 2),
        }

        for line, i in zip(self.lines, range(len(self.lines))):
            line.set_linewidth(4)
            if i < 4:
                ind = -1 if i == 3 else i
                x_vals = [coord_data[ind][0][0], coord_data[ind + 1][0][0]]
                y_vals = [coord_data[ind][0][1], coord_data[ind + 1][0][1]]
                z_vals = [coord_data[ind][0][2], coord_data[ind + 1][0][2]]
                line.set_data(x_vals, z_vals)
                line.set_3d_properties(y_vals)
            else:
                leg_num = line_to_leg_and_link_dict[i][0]
                link_num = line_to_leg_and_link_dict[i][1]
                x_vals = [coord_data[leg_num][link_num][0], coord_data[leg_num][link_num + 1][0]]
                y_vals = [coord_data[leg_num][link_num][1], coord_data[leg_num][link_num + 1][1]]
                z_vals = [coord_data[leg_num][link_num][2], coord_data[leg_num][link_num + 1][2]]
                line.set_data(x_vals, z_vals)
                line.set_3d_properties(y_vals)

        return self.lines


def main(args=None):
    rclpy.init(args=args)
    node = SpotMicroPlotNode()
    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
