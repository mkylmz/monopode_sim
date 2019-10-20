import sys

import pygame
from pygame.locals import *
from pygame.color import *
    

import myrobot

width, height = 1600,1600
def main():
    ### PyGame init
    pygame.init()
    screen = pygame.display.set_mode((width,height)) 
    clock = pygame.time.Clock()
    running = True
    font = pygame.font.SysFont("Arial", 16)
    
    ground = myrobot.draw_ground(screen)
    robot = myrobot.robot(screen,40,50,150,0,500)
    robot.draw(screen)
    
    
    while running:
        for event in pygame.event.get():
            if event.type == QUIT or \
                event.type == KEYDOWN and (event.key in [K_ESCAPE, K_q]):  
                running = False
            #elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            #    start_time = pygame.time.get_ticks()
            elif event.type == KEYDOWN and event.key == K_p:
                pygame.image.save(screen, "arrows.png")
            #elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            #    end_time = pygame.time.get_ticks()
            
        #keys = pygame.key.get_pressed()
        #speed = 2.5
        #if (keys[K_UP]):
        #    cannon_body.position += Vec2d(0,1) * speed
        #if (keys[K_DOWN]):
        #    cannon_body.position += Vec2d(0,-1) * speed
        #if (keys[K_LEFT]):
        #    cannon_body.position += Vec2d(-1,0) * speed
        #if (keys[K_RIGHT]):
        #    cannon_body.position += Vec2d(1,0) * speed
            
        ### Clear screen
        screen.fill(pygame.color.THECOLORS["lightblue"])

        # Physics Stuff
        robot.update()

        # Draw Stuff
        ground = myrobot.draw_ground(screen)
        robot.draw(screen)

                
        # Info and flip screen
        screen.blit(font.render("fps: " + str(clock.get_fps()), 1, THECOLORS["white"]), (0,0))
        screen.blit(font.render("Press ESC or Q to quit", 1, THECOLORS["white"]), (5,height - 20))
        
        pygame.display.flip()
        pygame.display.update()
        
        ### Update physics
        fps = 30
        dt = 1./fps
        
        clock.tick(fps)

if __name__ == '__main__':
    sys.exit(main())