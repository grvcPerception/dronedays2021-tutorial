#!/usr/bin/env python
#pip install pygame

import rospy
from std_msgs.msg import Int8
import pygame

def talker():
    rospy.init_node('teleop', anonymous=True)
    pub = rospy.Publisher('/keyboard', Int8, queue_size=1)
    rate = rospy.Rate(100)

    pygame.init()
    pygame.display.set_caption('UGV teleop')
    screen = pygame.display.set_mode((350, 130))

    textList = ["w: Increase speed", "s: Decrease speed", "a: Turn steering wheel to the rigth", "d: Turn steering wheel to the left"]
    k = 10
    for t in textList:
        font = pygame.font.Font(pygame.font.get_default_font(), 18)
        text = font.render(t, True, (255, 255, 255))
        screen.blit(text, dest=(10,k))
        pygame.display.update()
        k += 30

    while not rospy.is_shutdown():
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    pub.publish(ord('w'))
                if event.key == pygame.K_s:
                    pub.publish(ord('s'))
                if event.key == pygame.K_a:
                    pub.publish(ord('a'))
                if event.key == pygame.K_d:
                    pub.publish(ord('d'))
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_w:
                    pub.publish(-ord('w'))
                if event.key == pygame.K_s:
                    pub.publish(-ord('s'))
                if event.key == pygame.K_a:
                    pub.publish(-ord('a'))
                if event.key == pygame.K_d:
                    pub.publish(-ord('d'))
            if event.type == pygame.QUIT:
                exit()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
