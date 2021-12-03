# from tkinter import *
# import tkinter
from tkinter import Tk, Canvas, font
import random
import time
import numpy as np
from PIL import ImageTk,Image

# Define useful parameters
size_of_board = 600
rows = 10
cols = 10
DELAY = 100
symbol_size = (size_of_board / 3 - size_of_board / 8) / 2
symbol_thickness = 2
PURPLE_COLOR = "#59118e"


class Scoreboard:
    def __init__(self):
        self.window = Tk()
        self.window.title("Robokeeper")
        self.canvas = Canvas(self.window, width=size_of_board*2, height=size_of_board)
        self.canvas.pack()
        # Input from user in form of clicks and keyboard
        # self.play_again()
        self.begin = False
        self.robot_score = 0
        self.human_score = 0

    def initialize_board(self):
        self.board = []
        for i in range(rows):
            for j in range(cols):
                self.board.append((i, j))

    def mainloop(self, human, robot):
        while True:
            self.window.update()
            self.display_scores(human, robot)
                

    def display_scores(self, human, robot):
        score = 5
        robot_text = "ROBOT \n"
        human_text = "HUMAN \n"

        # put gif image on canvas
        # pic's upper left corner (NW) on the canvas is at x=50 y=10

        self.canvas.create_text(
            size_of_board*2*0.3,
            size_of_board*0.25,
            font=("Helvetica 60"),
            fill=PURPLE_COLOR,
            text=human_text,
        )

        # score_text = str(score)
        self.canvas.create_text(
            size_of_board*2*0.7,
            size_of_board*0.25,
            font=("Helvetica 60"),
            fill=PURPLE_COLOR,
            text=robot_text,
        )

        self.canvas.create_text(
            size_of_board*2*0.3,
            size_of_board*0.65,
            font=("Helvetica 300"),
            fill=PURPLE_COLOR,
            text=str(human),
        )

        self.canvas.create_text(
            size_of_board*2*0.7,
            size_of_board*0.65,
            font=("Helvetica 300"),
            fill=PURPLE_COLOR,
            text=str(robot),
        )

game_instance = Scoreboard()
game_instance.mainloop(game_instance.human_score, game_instance.robot_score)