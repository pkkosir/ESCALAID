#!/usr/bin/env python

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
# from OpenGL.GLUT import glutSwapBuffers as swap_buffers
import pygame
from pygame.locals import *
import serial
import time

#ser = serial.Serial('/dev/tty.usbserial', 38400, timeout=1)
ser = serial.Serial('COM4', 38400, timeout=3)
#ser_1 = serial.Serial('COM4', 38400, timeout=1)

ax = ay = az = 0.0
ax_1 = ay_1 = az_1 = 0.0
yaw_mode = False

def resize(width, height):
    if height==0:
        height=1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
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
    glBegin(GL_QUADS)	
    glColor3f(0.0,1.0,0.0)
    glVertex3f( 1.0, 0.2,-1.0)
    glVertex3f(-1.0, 0.2,-1.0)		
    glVertex3f(-1.0, 0.2, 1.0)		
    glVertex3f( 1.0, 0.2, 1.0)		

    glColor3f(1.0,0.5,0.0)	
    glVertex3f( 1.0,-0.2, 1.0)
    glVertex3f(-1.0,-0.2, 1.0)		
    glVertex3f(-1.0,-0.2,-1.0)		
    glVertex3f( 1.0,-0.2,-1.0)		

    glColor3f(1.0,0.0,0.0)		
    glVertex3f( 1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)		
    glVertex3f(-1.0,-0.2, 1.0)		
    glVertex3f( 1.0,-0.2, 1.0)		

    glColor3f(1.0,1.0,0.0)	
    glVertex3f( 1.0,-0.2,-1.0)
    glVertex3f(-1.0,-0.2,-1.0)
    glVertex3f(-1.0, 0.2,-1.0)		
    glVertex3f( 1.0, 0.2,-1.0)		

    glColor3f(0.0,0.0,1.0)	
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2,-1.0)		
    glVertex3f(-1.0,-0.2,-1.0)		
    glVertex3f(-1.0,-0.2, 1.0)		

    glColor3f(1.0,0.0,1.0)	
    glVertex3f( 1.0, 0.2,-1.0)
    glVertex3f( 1.0, 0.2, 1.0)
    glVertex3f( 1.0,-0.2, 1.0)		
    glVertex3f( 1.0,-0.2,-1.0)		
    glEnd()	

def drawText(position, textString):     
    font = pygame.font.SysFont ("Courier", 18, True)
    textSurface = font.render(textString, True, (255,255,255,255), (0,0,0,255))     
    textData = pygame.image.tostring(textSurface, "RGBA", True)     
    glRasterPos3d(*position)     
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def draw():
    global rquad
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
    

    #osd_text = "pitch: " + str("{0:.2f}".format(ay)) + ", roll: " + str("{0:.2f}".format(ax)) + ", roll_1: " + str("{0:.2f}".format(ax_1))
    # if yaw_mode:
    #     osd_line = osd_text + ", yaw: " + str("{0:.2f}".format(az))
    # else:
    #    osd_line = osd_text

    
    ##################
    ###  OBJECT 1  ###
    ##################
    glLoadIdentity()  # resets the coordinate plane
    glPushMatrix()  # create object 1's transformation matrix
    glTranslatef(-2, 0, -7.0)  # translate base to left side of environment from origin
    
    # BEGIN Rotate based on serialcomm
    if yaw_mode:
        glRotatef(az, 0.0, 1.0, 0.0)  # Yaw,   rotate around y-axis

    else:
        glRotatef(0.0, 0.0, 1.0, 0.0)
    glRotatef(ay ,1.0,0.0,0.0)        # Pitch, rotate around x-axis
    glRotatef(-1*ax ,0.0,0.0,1.0)     # Roll,  rotate around z-axis
	# END Rotate based on serialcomm

    prism()  # create a rectangular prism at the position specified by the transformation matrix
    glPopMatrix()  # delete object 2's transformation matrix
    drawText((-1.5, 1.2,-7.0), "Prism 1")  # label

    ##################
    ###  OBJECT 2  ###
    ##################
    glLoadIdentity()  # resets the coordinate plane
    glPushMatrix()  # create object 2's transformation matrix
    glTranslatef(2, 0, -7.0)  # translate base to right side of environment from origin

    # BEGIN Rotate based on serialcomm
    if yaw_mode:
        glRotatef(az_1, 0.0, 1.0, 0.0)  # Yaw,   rotate around y-axis
    else:
        glRotatef(0.0, 0.0, 1.0, 0.0)

    glRotatef(ay_1 ,1.0,0.0,0.0)        # Pitch, rotate around x-axis
    glRotatef(-1*ax_1 ,0.0,0.0,1.0)     # Roll,  rotate around z-axis
	# END Rotate based on serialcomm

    prism()  # create a rectangular prism at the position specified by the transformation matrix
    glPopMatrix()  # delete object 2's transformation matrix
    drawText((1.5, 1.2,-7.0), "Prism 2")  # label
    osd_line = f"Object1 -- pitch: {ay:.2f}, roll: {ax:.2f} {f", yaw: {az:.2f}" if yaw_mode else ''}"


         
def read_data():
    global ax, ay, az, ax_1, ay_1, az_1
    #ax = ay = az = 0.0
    #ax_1 = ay_1 = az_1 = 0.0
    line_done = 0
    #line_done_1 = 0

    # request data by sending a dot
    ser.write(b".") #* encode string to bytes
    #ser_1.write(b".") #RAM
    #while not line_done:
    line = ser.readline() 
    angles = line.split(b", ")
    
    #line_1 = ser_1.readline() 
    #angles_1 = line_1.split(b", ")

    if len(angles) == 6:    
        ax = float(angles[0])
        ay = float(angles[1])
        az = float(angles[2])
        ax_1 = float(angles[3])
        ay_1 = float(angles[4])
        az_1 = float(angles[5])
        line_done = 1 
    # print(ax, ay, az, ax_1, ay_1, az_1)
    #if len(angles_1) == 3:    
    #    ax_1 = float(angles_1[0])
    #    ay_1 = float(angles_1[1])
    #    az_1 = float(angles_1[2])
    #    line_done_1 = 1 

def main():
    global yaw_mode
    
    video_flags = OPENGL|DOUBLEBUF
    
    pygame.init()
    screen = pygame.display.set_mode((1280,720), video_flags)
    pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
    resize(1280,720)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()
    while 1:

        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()  #* quit pygame properly
            break       
        if event.type == KEYDOWN and event.key == K_z:
            yaw_mode = not yaw_mode
            ser.write(b"z")
            #ser_1.write(b"z")
        read_data()
        draw()
      
        pygame.display.flip()
        frames = frames+1

    print ("fps:  %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks)))
    
    ser.close()
    #ser_1.close()

if __name__ == '__main__': main()

