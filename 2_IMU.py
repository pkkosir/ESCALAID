#!/usr/bin/env python

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import pygame
from pygame.locals import *
import serial

ser = serial.Serial("COM4", 38400, timeout=3)

ax0, ay0, az0 = 0, 0, 0
ax1, ay1, az1 = 0, 0, 0
yaw_mode = False


def resize(width, height):
    height = 1 if not height else height
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
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

    osd_line = (
        f"Object1 -- pitch: {ay0:.2f}, roll: {ax0:.2f} {f", yaw: {az0:.2f}" if yaw_mode else ''}\n"
        + f"Object2 -- pitch: {ay1:.2f}, roll: {ax1:.2f} {f", yaw: {az1:.2f}" if yaw_mode else ''}"
    )

    ##################
    ###  OBJECT 1  ###
    ##################
    glLoadIdentity()  # resets the coordinate plane
    glPushMatrix()  # create object 1's transformation matrix
    glTranslatef(-2, 0, -7.0)  # translate base to left side of environment from origin

    # BEGIN Rotate based on serialcomm
    if yaw_mode:
        glRotatef(az0, 0.0, 1.0, 0.0)  # Yaw,   rotate around y-axis
    else:
        glRotatef(0.0, 0.0, 1.0, 0.0)
    glRotatef(ay0, 1.0, 0.0, 0.0)  # Pitch, rotate around x-axis
    glRotatef(-1 * ax0, 0.0, 0.0, 1.0)  # Roll,  rotate around z-axis
    # END Rotate based on serialcomm

    prism()  # create a rectangular prism at the position specified by the transformation matrix
    glPopMatrix()  # delete object 2's transformation matrix
    drawText((-1.5, 1.2, -7.0), "Prism 1")  # label

    ##################
    ###  OBJECT 2  ###
    ##################
    glLoadIdentity()  # resets the coordinate plane
    glPushMatrix()  # create object 2's transformation matrix
    glTranslatef(2, 0, -7.0)  # translate base to right side of environment from origin

    # BEGIN Rotate based on serialcomm
    if yaw_mode:
        glRotatef(az1, 0.0, 1.0, 0.0)  # Yaw,   rotate around y-axis
    else:
        glRotatef(0.0, 0.0, 1.0, 0.0)

    glRotatef(ay1, 1.0, 0.0, 0.0)  # Pitch, rotate around x-axis
    glRotatef(-1 * ax1, 0.0, 0.0, 1.0)  # Roll,  rotate around z-axis
    # END Rotate based on serialcomm

    prism()  # create a rectangular prism at the position specified by the transformation matrix
    glPopMatrix()  # delete object 2's transformation matrix
    drawText((1.5, 1.2, -7.0), "Prism 2")  # label
    drawText((-2, -2, 2), osd_line)  # numerical imu data


def read_data():
    global ax0, ay0, az0, ax1, ay1, az1

    # request data by sending a dot
    ser.write(b".")  # * encode string to bytes
    line = ser.readline()
    angles = line.split(b", ")

    if len(angles) == 6:
        ax0 = float(angles[0])
        ay0 = float(angles[1])
        az0 = float(angles[2])
        ax1 = float(angles[3])
        ay1 = float(angles[4])
        az1 = float(angles[5])


def main():
    global yaw_mode

    video_flags = OPENGL | DOUBLEBUF

    pygame.init()
    screen = pygame.display.set_mode((1280, 720), video_flags)
    pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
    resize(1280, 720)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()

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
        frames += 1

    print("fps:  %d" % ((frames * 1000) / (pygame.time.get_ticks() - ticks)))

    ser.close()


if __name__ == "__main__":
    main()
