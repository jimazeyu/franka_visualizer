import viser
import viser.transforms as tf
from viser.extras import ViserUrdf

import roboticstoolbox as rtb
from spatialmath import SE3
import transforms3d.euler as euler
import numpy as np

import numpy as np

from typing import List

from pathlib import Path

import time

import open3d as o3d

import os

def main() -> None:
    server = viser.ViserServer()

    with server.add_gui_folder("Pointcloud") as folder:
        gui_pointcloud_url = server.add_gui_text(
            "Pointcloud URL",
            initial_value="pointcloud",
        )

        gui_load_pointcloud = server.add_gui_button("Load Pointcloud")

    with server.add_gui_folder("Panda FK") as folder:
        # add arm model
        urdf = ViserUrdf(server, urdf_path = Path("./panda/panda.urdf"), root_node_name = "/panda_base")

        # Create joint angle sliders.
        gui_joints: List[viser.GuiInputHandle[float]] = []
        initial_angles: List[float] = []
        for joint_name, (lower, upper) in urdf.get_actuated_joint_limits().items():
            lower = lower if lower is not None else -np.pi
            upper = upper if upper is not None else np.pi

            initial_angle = 0.0 if lower < 0 and upper > 0 else (lower + upper) / 2.0
            slider = server.add_gui_slider(
                label=joint_name,
                min=lower,
                max=upper,
                step=1e-3,
                initial_value=initial_angle,
            )
            slider.on_update(  # When sliders move, we update the URDF configuration.
                lambda _: urdf.update_cfg(np.array([gui.value for gui in gui_joints]))
            )

            gui_joints.append(slider)
            initial_angles.append(initial_angle)

        # Apply initial joint angles.
        urdf.update_cfg(np.array([gui.value for gui in gui_joints]))

    with server.add_gui_folder("Panda IK") as folder:
        # End effector pose
        gui_x = server.add_gui_text(
            "X",
            initial_value="0.4",
        )       
        gui_y = server.add_gui_text(
            "Y",
            initial_value="0.0",
        )
        gui_z = server.add_gui_text(
            "Z",
            initial_value="0.4",
        )
        gui_roll = server.add_gui_text(
            "Roll",
            initial_value="3.1415926",
        )
        gui_pitch = server.add_gui_text(
            "Pitch",
            initial_value="0.0",
        )
        gui_yaw = server.add_gui_text(
            "Yaw",
            initial_value="0.0",
        )


        # Create grasp button.
        arm_grasp_button = server.add_gui_button("Grasp")

        # Create joint reset button.
        arm_reset_button = server.add_gui_button("Reset")

    @arm_grasp_button.on_click
    def _(_) -> None:
        best_sol = None
        best_error = 10086 # error is the biggest difference between the current joint angles and the previous joint angles
        now_joint_angles = [float(gui.value) for gui in gui_joints]

        for i in range(20):
            # calculate IK
            robot = rtb.models.Panda()
            roll = float(gui_roll.value)
            pitch = float(gui_pitch.value)
            yaw = float(gui_yaw.value)
            translation = [float(gui_x.value), float(gui_y.value), float(gui_z.value)]
            Tep = SE3.Trans(translation) * SE3.RPY([roll, pitch, yaw])
            sol, success, iterations, searches, residual = robot.ik_NR(Tep)         # solve IK
            if success:
                error = np.max(np.abs(np.array(sol[0]) - np.array(now_joint_angles)))
                # print("error", error)
                if best_sol is None:
                    best_sol = sol
                    best_error = error
                elif error < best_error:
                    best_sol = sol
                    best_error = error                    

        if best_sol is None:
            print("No solution found")
            return
        
        print("Tep", Tep)
        print("joint angles", best_sol)


        frame_end_effector = server.add_frame(
            name="end_effector",
            wxyz=tf.SO3.from_rpy_radians(roll, pitch, yaw).wxyz,
            position=tuple(translation),
            show_axes=True,
            axes_length=0.1,
            axes_radius=0.005
        )

        for i, angle in enumerate(best_sol):
            gui_joints[i].value = angle

    @arm_reset_button.on_click
    def _(_):
        for g, initial_angle in zip(gui_joints, initial_angles):
            g.value = initial_angle

    @gui_load_pointcloud.on_click
    def _(_) -> None:
        pointcloud_url = gui_pointcloud_url.value + ".ply"
        print('Loading point cloud from:', pointcloud_url)
        # load point cloud
        if not os.path.exists(pointcloud_url):
            print('Pointcloud file not found')
            return
        pcd = o3d.io.read_point_cloud(pointcloud_url)
        vertices = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        print('Point cloud loaded')
        # add point cloud to server
        server.add_point_cloud(
            "full_pointcloud",
            points=vertices,
            colors=colors,
            point_size=0.002,
            position=(0, 0, 0)
        )

    while True:
        time.sleep(0.01)

if __name__ == "__main__":
    main()