import math
import numpy as np

pi = math.pi

def PointsInCircum(r=20,n=100):
    return [(math.cos(2*pi/n*x)*r,math.sin(2*pi/n*x)*r) for x in range(0,n+1)]

def infinityCircum(n=100):
    t = np.linspace(0, 2*np.pi, n)
    x = np.sin(t)
    y = np.sin(t)*np.cos(t)
    res = zip(x, y)
    return res

if __name__=="__main__":
    z = PointsInCircum(3, 20)
    print(z)
    z = infinityCircum(n = 20)
    print(z)