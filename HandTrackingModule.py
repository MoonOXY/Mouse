"""
Hand Tracing Module
By: Murtaza Hassan
Youtube: http://www.youtube.com/c/MurtazasWorkshopRoboticsandAI
Website: https://www.computervision.zone/
"""

import cv2
import mediapipe as mp
import time
import math
import numpy as np
import pyautogui



class handDetector():
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands,
                                        self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]
        self.open_hand = [1, 1, 1, 1, 1]  # 手掌张开时每个手指的状态
        self.closed_hand = [0, 0, 0, 0, 0]  # 握拳时每个手指的状态
        self.left_mouse_hand_move = [0, 1, 1, 0, 0] # 比耶手势
        self.left_mouse_hand_click = [0, 1, 0, 0, 0] # 点击手势
        self.right_mouse_hand = [0, 0, 1, 1, 1] # 三手势

        self.flag = False
        self.is_open_hand = False
        self.left_mouse = False
        self.left_mouse_click = False
        self.right_mouse = False

    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        # print(results.multi_hand_landmarks)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)

        return img

    def findPosition(self, img, handNo=0, draw=True):
        xList = []
        yList = []
        bbox = []
        self.lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                # print(id, lm)
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                xList.append(cx)
                yList.append(cy)
                # print(id, cx, cy)
                self.lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 5, (255, 0, 255), cv2.FILLED)

            xmin, xmax = min(xList), max(xList)
            ymin, ymax = min(yList), max(yList)
            bbox = xmin, ymin, xmax, ymax

            if draw:
                cv2.rectangle(img, (xmin - 20, ymin - 20), (xmax + 20, ymax + 20),
                              (0, 255, 0), 2)

        return self.lmList, bbox

    def fingersUp(self):
        if len(self.lmList) == 0:
            fingers = [-1, -1, -1, -1, -1]
            return fingers
        fingers = []
        # Thumb
        #print("=======")
        #print(self.lmList)
        #print(self.tipIds[0])
        if self.lmList[self.tipIds[0]][1] > self.lmList[self.tipIds[0] - 2][1]:
            fingers.append(1)
        else:
            fingers.append(0)

        # Fingers
        for id in range(1, 5):

            if self.lmList[self.tipIds[id]][2] < self.lmList[self.tipIds[id] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)

        # totalFingers = fingers.count(1)

        return fingers
    
    def ActionCatch(self, fingers, lmList):
        if len(lmList) != 0:
            if self.flag == False:
                    if fingers == self.open_hand:
                        self.is_open_hand = True
                    if self.is_open_hand:
                        if fingers == self.closed_hand:
                            self.is_open_hand = False
                            self.flag = True
            else:
                if fingers == self.open_hand:
                    self.is_open_hand = True
                    self.flag = False
        else:
            self.flag = False

    def Actionmouse(self, fingers):
        if self.flag:
            if fingers == self.left_mouse_hand_move:
                if self.left_mouse == False:
                    #left mouse down
                    print("left mouse down")
                    pyautogui.mouseDown(button='left')
                    self.left_mouse = True
                    self.right_mouse = False
            elif fingers == self.right_mouse_hand:
                if self.right_mouse == False:
                    #right mouse down
                    print("right mouse click")
                    pyautogui.rightClick()
                    self.left_mouse = False
                    self.right_mouse = True
            elif fingers == self.left_mouse_hand_click:
                if self.left_mouse_click == False:
                    pyautogui.leftClick()
            elif fingers == self.closed_hand:
                if self.left_mouse == True:
                    #left mouse up
                    print("left mouse up")
                    pyautogui.mouseUp(button='left')
                    self.left_mouse = False
                if self.right_mouse == True:
                    #right mouse up
                    print("right mouse reset")
                    self.right_mouse = False
                if self.left_mouse_click== True:
                    #right mouse up
                    print("left mouse reset")
                    self.left_mouse_click = False
            else :
                self.left_mouse = False
                self.right_mouse = False

            



    def findDistance(self, p1, p2, img, draw=True,r=15, t=3):
        x1, y1 = self.lmList[p1][1:]
        x2, y2 = self.lmList[p2][1:]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        if draw:
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), t)
            cv2.circle(img, (x1, y1), r, (255, 0, 255), cv2.FILLED)
            cv2.circle(img, (x2, y2), r, (255, 0, 255), cv2.FILLED)
            cv2.circle(img, (cx, cy), r, (0, 0, 255), cv2.FILLED)
        length = math.hypot(x2 - x1, y2 - y1)

        return length, img, [x1, y1, x2, y2, cx, cy]


def main():
    pTime = 0
    cTime = 0
    cap = cv2.VideoCapture(1)
    detector = handDetector()
    while True:
        success, img = cap.read()
        img = detector.findHands(img)
        lmList, bbox = detector.findPosition(img)
        if len(lmList) != 0:
            print(lmList[4])

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime

        cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3,
                    (255, 0, 255), 3)

        cv2.imshow("Image", img)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()