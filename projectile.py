import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
g = 9.81
step = 0.1
theta = 45

class Projectile:
    def __init__(self, m, p, v, a, f):
        self.m = m
        self.p = p
        self.p_r = [p]
        self.v = v
        self.v_r = [v]
        self.a = a
        self.a_r = [a]
        self.f = f
        self.nf = sum(f)
        self.time = 0
        print("Initial Position:", self.p)
        print("Initial Velocity:", self.v)
        print("Initial Acceleration:", self.a)
        print("Initial Net Force:", self.nf)

    def time_set(self, current_time):
        self.time = current_time
        return self.time

    def apply_force(self):
        self.nf = sum(self.f)
        self.a = np.add(self.a, self.nf / self.m)
        return self.a

    def delta_time(self, current_time):
        return current_time - self.time

    def velocity_calc(self, current_time):
        delta_time = self.delta_time(current_time)
        self.v = self.v + self.a * delta_time
        self.v_r.append(self.v)
        return delta_time, self.v

    def position_calc(self, current_time):
        delta_time = self.delta_time(current_time)
        self.p = self.p + self.v * delta_time + 0.5 * self.a * delta_time ** 2
        self.velocity_calc(current_time)
        self.p_r.append(self.p)
        return delta_time, self.p

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
velocity = np.array([0, 10, 0])
acceleration = np.array([0, 0, 0])
ball = Projectile(1, position, velocity, acceleration, [])

origin = np.array([0, 0, 0])
dim = np.array([10, 10, 10])
grav = np.array([0, 0, -10])
sys = System(origin, dim, grav, [ball])

t = 0
ball.apply_force()

for n in range(100):
    ball.position_calc(t)
    ball.time_set(t)
    t += step


# Generate some sample 3D data

num_points = 100
x = [i[0] for i in ball.p_r]
y = [i[1] for i in ball.p_r]
z = [i[2] for i in ball.p_r]

# Create a figure and a 3D axes object
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Create the 3D scatter plot
ax.scatter(x, y, z)

# Set axis labels
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')

# Set a title for the plot
ax.set_title('Basic 3D Scatter Plot')

# Display the plot
plt.show()