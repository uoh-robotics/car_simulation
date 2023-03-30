import argparse
import mujoco as mj
# from mujoco import viewer
from mujoco.glfw import glfw
import numpy as np
import os
# from scipy.spatial.transform import Rotation as r


class MJCar:
    def __init__(self, xml_file):
        self.xml_file = xml_file
        if os.path.isabs(self.xml_file):
            self.xml_file = os.path.abspath(self.xml_file)
        self._init_vars()

    def _init_vars(self):
        self.velocity = 5
        self.simulation_time = 1000  # simulation time
        self.stop_simulation = False
        # set to 1 to print camera config
        # this is useful for initializing view of the model)
        self.print_camera_config = 0

        # For callback functions
        self.button_left = False
        self.button_middle = False
        self.button_right = False
        self.lastx = 0
        self.lasty = 0
        self.velocity = 5
        self.overlay = {}
        self.window = None
        self.ov_positions = {"topleft": mj.mjtGridPos.mjGRID_TOPLEFT,
                             "topright": mj.mjtGridPos.mjGRID_TOPRIGHT,
                             "bottomleft": mj.mjtGridPos.mjGRID_BOTTOMLEFT,
                             "bottomright": mj.mjtGridPos.mjGRID_BOTTOMRIGHT}

    def _init_model(self):
        # MuJoCo data structures
        self.model = mj.MjModel.from_xml_path(self.xml_file)  # MuJoCo model
        self.data = mj.MjData(self.model)                     # MuJoCo data
        self.camera = mj.MjvCamera()                          # Abstract camera
        self.opt = mj.MjvOption()  # visualization options

    def _init_glfw(self):
        # Init GLFW, create window, make OpenGL context current, request v-sync
        glfw.init()
        self.window = glfw.create_window(1200, 900, "Demo", None, None)
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

    def _init_camera(self):
        # Camera configuration
        self.camera.azimuth = 89.8000000000001
        self.camera.elevation = -38.600000000000186
        self.camera.distance = 10.77018095510105
        self.camera.lookat = np.array([0.0, 0.0, 0.0])

    def _init_vis(self):
        # initialize visualization data structures
        mj.mjv_defaultCamera(self.camera)
        mj.mjv_defaultOption(self.opt)
        self.scene = mj.MjvScene(self.model, maxgeom=10000)
        self.context = mj.MjrContext(self.model, mj.mjtFontScale.mjFONTSCALE_150.value)

    def _init_mouse_keyboard(self):
        # install GLFW mouse and keyboard callbacks
        glfw.set_key_callback(self.window, self.keyboard)
        glfw.set_cursor_pos_callback(self.window, self.mouse_move)
        glfw.set_mouse_button_callback(self.window, self.mouse_button)
        glfw.set_scroll_callback(self.window, self.scroll)

    # def add_overlay(self, gridpos, text1, text2):
    #     if gridpos not in self.overlay:
    #         self.overlay[gridpos] = ["", ""]
    #     self.overlay[gridpos][0] += text1 + "\n"
    #     self.overlay[gridpos][1] += text2 + "\n"

    # def ppipcreate_overlay(self, model, data):
    #     topleft = mj.mjtGridPos.mjGRID_TOPLEFT
    #     topright = mj.mjtGridPos.mjGRID_TOPRIGHT
    #     bottomleft = mj.mjtGridPos.mjGRID_BOTTOMLEFT
    #     bottomright = mj.mjtGridPos.mjGRID_BOTTOMRIGHT

    #     # (mj.mjtFont.mjFONT_BIG, mj.mjmjGRID_TOPLEFT, smallrect,
    #     #         topleftlabel.c_str(), nullptr, &this->con)
    #     self.add_overlay(bottomleft, "Restart", 'r')
    #     self.add_overlay(bottomleft, "Start", 's')
    #     self.add_overlay(bottomleft, "Time", '%.2f' % data.time)
    #     self.add_overlay(topleft, "velocity ", '%.2f' % self.velocity)

    def add_overlay(self, pos, text1, text2):
        font = mj.mjtFont.mjFONT_SHADOW
        pos = self.ov_positions[pos]
        mj.mjr_overlay(font, pos, self.viewport, text1, text2, self.context)

    def default_overlay(self):
        self.add_overlay("bottomleft",
                         "\n".join(["Restart: r", "Start: s", f"Time: {self.data.time}"]),
                         None)
        self.add_overlay("topleft", "velocity", f"{self.velocity}")

    def keyboard(self, window, key, scancode, act, mods):
        _wire_frame = False
        _joints = False
        model = self.model
        data = self.data
        scene = self.scene
        viewport = self.viewport
        # if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        #     mj.mj_resetData(model, data)
        #     mj.mj_forward(model, data)
        if (act == glfw.PRESS and key == glfw.KEY_R):
            mj.mj_resetData(model, data)
            mj.mj_forward(model, data)

        # press the space key to completely stop the movement
        if key == glfw.KEY_SPACE:
            data.ctrl = np.zeros(len(data.ctrl))

        # press the up arrow to start the motion and increase the speed by one unit if clicked again
        if key == glfw.KEY_UP:
            data.ctrl[0] += 0.05
            data.ctrl[1] += 0.05
            data.ctrl[2] += 0.05
            data.ctrl[3] += 0.05
            data.ctrl[4] += 0.05
            # data.ctrl[5] += 0.1
            # data.ctrl[6] += 0.1
            # data.ctrl[7] += 0.1

        # press the down arrow to decrease the speed by one unit
        if key == glfw.KEY_DOWN:
            data.ctrl[0] -= 0.05
            data.ctrl[1] -= 0.05
            data.ctrl[2] -= 0.05
            data.ctrl[3] -= 0.05
            data.ctrl[4] -= 0.05
            # data.ctrl[5] -= 0.1
            # data.ctrl[6] -= 0.1
            # data.ctrl[7] -= 0.1

        # press the right arrow to turn the body right
        # if key == glfw.KEY_RIGHT:
            data.ctrl[1] += 0.1
            # data.ctrl[3] += 0.1
            data.ctrl[0] += 0.1
            # data.ctrl[2] -= 1

        # press the left arrow to turn the body left
        if key == glfw.KEY_LEFT:
            # data.ctrl[0] -= 0.1
            data.ctrl[2] += 0.1
            # data.ctrl[1] -= 0.1
            data.ctrl[3] -= 1

        # press W key to show the frames in wireframe
        if key == glfw.KEY_W:
            _wire_frame = not _wire_frame
            scene.flags[mj.mjtRndFlag.mjRND_WIREFRAME] = _wire_frame

        # press J key to display the joints
        if key == glfw.KEY_J:
            _joints = not _joints
            viewport.flags[mj.mjtVisFlag.mjVIS_JOINT] = _joints

        if (glfw.MOD_CONTROL and key == glfw.KEY_C) or glfw.KEY_Q:
            self.stop_simulation = True

    def init_controller(self, model, data):
        # initialize the controller here. This function is called once, in the beginning
        pass

    def controller(self, model, data):
        # put the controller here. This function is called inside the simulation

        # data.ctrl[0] = velocity
        # vx = 0
        # vy = 0
        # vz = velocity
        # v = np.sqrt(vx**2+vy**2+vz**2)
        # c = 0.5
        # data.xfrc_applied[1][0] = -c*vx*v
        # data.xfrc_applied[1][1] = -c*vy*v
        # data.xfrc_applied[1][2] = -c*vz*v
        pass

    def mouse_button(self, window, button, act, mods):
        # update button state
        button_left = self.button_left
        button_middle = self.button_middle
        button_right = self.button_right

        button_left = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
        button_middle = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
        button_right = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

        # update mouse position
        glfw.get_cursor_pos(window)

    def mouse_move(self, window, xpos, ypos):
        # compute mouse displacement, save
        lastx = self.lastx
        lasty = self.lasty
        button_left = self.button_left
        button_middle = self.button_middle
        button_right = self.button_right
        model = self.model
        scene = self.scene
        cam = self.camera

        dx = xpos - lastx
        dy = ypos - lasty
        lastx = xpos
        lasty = ypos

        # no buttons down: nothing to do
        if (not button_left) and (not button_middle) and (not button_right):
            return

        # get current window size
        width, height = glfw.get_window_size(window)

        # get shift key state
        PRESS_LEFT_SHIFT = glfw.get_key(
            window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
        PRESS_RIGHT_SHIFT = glfw.get_key(
            window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
        mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

        # determine action based on mouse button
        if button_right:
            if mod_shift:
                action = mj.mjtMouse.mjMOUSE_MOVE_H
            else:
                action = mj.mjtMouse.mjMOUSE_MOVE_V
        elif button_left:
            if mod_shift:
                action = mj.mjtMouse.mjMOUSE_ROTATE_H
            else:
                action = mj.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mj.mjtMouse.mjMOUSE_ZOOM

        mj.mjv_moveCamera(model, action, dx/height,
                          dy/height, scene, cam)

    def scroll(self, xoffset, yoffset):
        action = mj.mjtMouse.mjMOUSE_ZOOM
        model = self.model
        scene = self.scene
        cam = self.camera
        mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                          yoffset, scene, cam)

    def start(self):
        self._init_vars()
        self._init_model()
        self._init_glfw()
        self._init_vis()
        self._init_mouse_keyboard()
        self._init_camera()
        self.init_controller(self.model, self.data)
        mj.set_mjcb_control(self.controller)


        window = self.window
        model = self.model
        data = self.data
        cam = self.camera
        while not glfw.window_should_close(window):
            if self.stop_simulation:
                break

            time_prev = data.time

            while (data.time - time_prev < 1.0/100.0):
                mj.mj_step(model, data)
                # print(data.time, data.ctrl)

            # xyz pos of free joint
            # print(data.qpos[0], data.qpos[1], data.qpos[2])

            if (data.time >= self.simulation_time):
                break

            # get framebuffer viewport
            viewport_width, viewport_height = glfw.get_framebuffer_size(window)
            self.viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            # print camera configuration (help to initialize the view)
            if (self.print_camera_config):
                print(f'cam.azimuth={cam.azimuth}, cam.elevation={cam.elevation}'
                      f'cam.distance={cam.distance}')
                print(f'cam.lookat={np.array([cam.lookat[0], cam.lookat[1],cam.lookat[2]])}')

            # Move the camera wrt to the body
            cam.lookat[0] = data.qpos[0]
            # Update scene and render
            mj.mjv_updateScene(model, data, self.opt, None, cam,
                               mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(self.viewport, self.scene, self.context)

            self.default_overlay()

            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()
        glfw.terminate()



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--xml-file", default="democar.xml")
    parser.add_argument("--meshdir")
    args = parser.parse_args()
    sim = MJCar(args.xml_file)
    sim.start()
