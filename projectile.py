import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from sympy import symbols, sympify, diff
g = 9.81
step = 0.1
theta = 45
t = symbols("t")
z_v = np.array([0,0,0])

class Projectile:
    def __init__(self, p, td, m, v = z_v, a = z_v, f = z_v):
        
        self.p = p
        self.td = td
        self.time = 0
        self.m = m
        self.nf = sum(f)
        
        if not td:
            self.v = v
            self.a = a
            self.f = f
            self.p_r = [p]
            self.v_r = [v]
            self.a_r = [a]
            
        else:
            self.v = np.array([diff(i) for i in self.p])
            self.a = np.array([diff(i) for i in self.v])
            self.f = self.a / self.m
            self.p_r = [[float(sympify(i).subs(t, self.time)) for i in self.p]]
            self.v_r = [[float(sympify(i).subs(t, self.time)) for i in self.v]]
            self.a_r = [[float(sympify(i).subs(t, self.time)) for i in self.a]]

        print("Initial Position:", self.p)
        print("Initial Velocity:", self.v)
        print("Initial Acceleration:", self.a)
        print("Initial Net Force:", self.nf)

    def time_set(self, current_time):
        self.time = current_time
        return self.time

    def delta_time(self, current_time):
        return current_time - self.time

    def position_calc(self, current_time):
        if not self.td:
            delta_time = self.delta_time(current_time)
            self.p = self.p + self.v * delta_time + 0.5 * self.a * delta_time ** 2
            self.velocity_calc(current_time)
            self.p_r.append(self.p)
            self.time_set(current_time)
            return delta_time, self.p
        p = [float(sympify(i).subs(t, current_time)) for i in self.p]
        self.p_r.append(p)
        return current_time, self.p
    
    def velocity_calc(self, current_time):
        if not self.td:
            delta_time = self.delta_time(current_time)
            self.v = self.v + self.a * delta_time
            self.v_r.append(self.v)
            self.time_set(current_time)
            return delta_time, self.v

    def apply_force(self):
        self.nf = sum(self.f)
        self.a = np.add(self.a, self.nf / self.m)
        return self.a

class System:
    def __init__(self, origin, dimensions, gravity, objects):
        self.origin = origin
        self.dimensions = dimensions
        self.gravity = gravity
        self.objects = objects 

    def apply_gravity(self):
        for object in self.objects:
            object.f.append(self.gravity)
            print(object.apply_force())      

position = np.array([0, 0, 0])
velocity = np.array([0, 1, 0])
acceleration = np.array([0, 0, 0])
discrete_ball = Projectile(position, False, 1, velocity, acceleration)

position = np.array([t, t, t])
continuous_ball = Projectile(position, True, 1)

# origin = np.array([0, 0, 0])
# dim = np.array([10, 10, 10])
# grav = np.array([0, 0, -10])
# sys = System(origin, dim, grav, [ball])

global_time = 0

for n in range(10):
    print(discrete_ball.position_calc(global_time))
    continuous_ball.position_calc(global_time)
    global_time += step

x1 = [i[0] for i in continuous_ball.p_r]
y1 = [i[1] for i in continuous_ball.p_r]
z1 = [i[2] for i in continuous_ball.p_r]

x2 = [i[0] for i in discrete_ball.p_r]
y2 = [i[1] for i in discrete_ball.p_r]
z2 = [i[2] for i in discrete_ball.p_r]

print(x1)
print(x2)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(x1, y1, z1, ["red"])
ax.scatter(x2, y2, z2, ["blue"])

ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')

plt.show()