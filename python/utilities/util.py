from collections import deque
import numpy as np

def custon_logg(y_true,  y_pred):
    pass

def first_load_deques(y0, u0, nd, dd):
    u_deque = deque()
    y_deque = deque()
    for _ in range(nd):
        u_deque.append(u0)
    for _ in range(dd):
        y_deque.append(y0)
    return u_deque, y_deque

def roll_deque(deq, value):
    deq.pop()
    deq.appendleft(value)
    return deq

def normalize_and_clip_angle(angle, min_, max_):
    #while (angle < min_):
    #    angle += 180
    #while (angle > max_):
    #    angle -= 180
    angle = (angle + 180.) % (360.) - 180.

    angle = np.clip(angle, min_, max_)
    return angle
