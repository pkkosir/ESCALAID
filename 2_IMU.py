#!/usr/bin/env python

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import pygame
from pygame.locals import *
import serial


class Axes:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z

    def update(self, x: float = None, y: float = None, z: float = None):
        self.x = float(x) if x else self.x
        self.y = float(y) if y else self.y
        self.z = float(z) if z else self.z


# Global variables
ser = serial.Serial("COM4", 38400, timeout=3)
yaw_mode = False
obj0, obj1 = Axes(), Axes()


def resize_window(width, height):
    height = 1 if not height else height
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def gl_env_init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def prism() -> None:
    """Create a rectangular prism with coloured sides"""
    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()


def drawText(position, textString):
    font = pygame.font.SysFont("Courier", 18, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(
        textSurface.get_width(),
        textSurface.get_height(),
        GL_RGBA,
        GL_UNSIGNED_BYTE,
        textData,
    )


def draw():
    global rquad
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    drawText(
        (-2, -2, -7.0),
        f"Object1 -- pitch: {obj0.y:.2f}, roll: {obj0.x:.2f} {f', yaw: {obj0.z:.2f}' if yaw_mode else ''}",
    )
    drawText(
        (-2, -2.2, -7.0),
        f"Object2 -- pitch: {obj1.y:.2f}, roll: {obj1.x:.2f} {f', yaw: {obj1.z:.2f}' if yaw_mode else ''}",
    )

    ##################
    ###  OBJECT 1  ###
    ##################
    glLoadIdentity()  # resets the coordinate plane
    glPushMatrix()  # create object 1's transformation matrix
    glTranslatef(-2, 0, -7.0)  # translate base to left side of environment from origin

    # BEGIN Rotate based on serialcomm
    if yaw_mode:
        glRotatef(obj0.z, 0.0, 1.0, 0.0)  # Yaw,   rotate around y-axis
    else:
        glRotatef(0.0, 0.0, 1.0, 0.0)
    glRotatef(obj0.y, 1.0, 0.0, 0.0)  # Pitch, rotate around x-axis
    glRotatef(-1 * obj0.x, 0.0, 0.0, 1.0)  # Roll,  rotate around z-axis
    # END Rotate based on serialcomm

    prism()  # create a rectangular prism at the position specified by the transformation matrix
    glPopMatrix()  # delete object 2's transformation matrix
    drawText((-2, 1.2, -7.0), "Prism 1")  # label

    ##################
    ###  OBJECT 2  ###
    ##################
    glLoadIdentity()  # resets the coordinate plane
    glPushMatrix()  # create object 2's transformation matrix
    glTranslatef(2, 0, -7.0)  # translate base to right side of environment from origin

    # BEGIN Rotate based on serialcomm
    if yaw_mode:
        glRotatef(obj1.z, 0.0, 1.0, 0.0)  # Yaw,   rotate around y-axis
    else:
        glRotatef(0.0, 0.0, 1.0, 0.0)

    glRotatef(obj1.y, 1.0, 0.0, 0.0)  # Pitch, rotate around x-axis
    glRotatef(-1 * obj1.x, 0.0, 0.0, 1.0)  # Roll,  rotate around z-axis
    # END Rotate based on serialcomm

    prism()  # create a rectangular prism at the position specified by the transformation matrix
    glPopMatrix()  # delete object 2's transformation matrix
    drawText((1.5, 1.2, -7.0), "Prism 2")  # label


def read_data():
    global obj0, obj1

    # request data by sending a dot
    ser.write(b".")  # * encode string to bytes
    line = ser.readline()
    angles = line.split(b", ")

    if len(angles) == 6:
        obj0.update(*angles[:3])
        obj1.update(*angles[3:])


def main():
    global yaw_mode

    video_flags = OPENGL | DOUBLEBUF

    pygame.init()
    screen = pygame.display.set_mode((1280, 720), video_flags)
    pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
    resize_window(1280, 720)
    gl_env_init()

    while True:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()  # quit pygame properly
            break
        if event.type == KEYDOWN and event.key == K_z:
            yaw_mode = not yaw_mode
            ser.write(b"z")

        read_data()
        draw()

        pygame.display.flip()

    ser.close()


if __name__ == "__main__":
    main()
