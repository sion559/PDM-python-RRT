import numpy as np
import math
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
import sys

#ORIGINAL
class GUI():
    # 'quad_list' is a dictionary of format: quad_list = {'quad_1_name':{'position':quad_1_position,'orientation':quad_1_orientation,'arm_span':quad_1_arm_span}, ...}
    def __init__(self, quads, obs):
        self.quads = quads
        self.obstacles = obs
        self.fig = plt.figure()
        self.ax = Axes3D.Axes3D(self.fig)
        self.ax.set_xlim3d([-2.0, 2.0])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([-2.0, 2.0])
        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([0, 5.0])
        self.ax.set_zlabel('Z')
        self.ax.set_title('Quadcopter Simulation')
        self.init_plot()
        self.fig.canvas.mpl_connect('key_press_event', self.keypress_routine)

    def rotation_matrix(self,angles):
        ct = math.cos(angles[0])
        cp = math.cos(angles[1])
        cg = math.cos(angles[2])
        st = math.sin(angles[0])
        sp = math.sin(angles[1])
        sg = math.sin(angles[2])
        R_x = np.array([[1,0,0],[0,ct,-st],[0,st,ct]])
        R_y = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
        R_z = np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])
        R = np.dot(R_z, np.dot( R_y, R_x ))
        return R

    def init_plot(self):
        for key in self.quads:
            self.quads[key]['l1'], = self.ax.plot([],[],[],color='blue',linewidth=3,antialiased=False)
            self.quads[key]['l2'], = self.ax.plot([],[],[],color='red',linewidth=3,antialiased=False)
            self.quads[key]['hub'], = self.ax.plot([],[],[],marker='o',color='green', markersize=6,antialiased=False)
            # prepare some coordinates
            for x,y,z,dx,dy,dz in self.obstacles:
                self.plot_opaque_cube(x,y,z,dx,dy,dz)

    def update(self):
        for key in self.quads:
            R = self.rotation_matrix(self.quads[key]['orientation'])
            L = self.quads[key]['L']
            points = np.array([ [-L,0,0], [L,0,0], [0,-L,0], [0,L,0], [0,0,0], [0,0,0] ]).T
            points = np.dot(R,points)
            points[0,:] += self.quads[key]['position'][0]
            points[1,:] += self.quads[key]['position'][1]
            points[2,:] += self.quads[key]['position'][2]
            self.quads[key]['l1'].set_data(points[0,0:2],points[1,0:2])
            self.quads[key]['l1'].set_3d_properties(points[2,0:2])
            self.quads[key]['l2'].set_data(points[0,2:4],points[1,2:4])
            self.quads[key]['l2'].set_3d_properties(points[2,2:4])
            self.quads[key]['hub'].set_data(points[0,5],points[1,5])
            self.quads[key]['hub'].set_3d_properties(points[2,5])
            #set ranges to follow
            y = [None]*2
            y[0] = self.quads[key]['position'][0] - 2
            y[1] = self.quads[key]['position'][0] + 2
            self.ax.set_xlim3d(y)
            y[0] = self.quads[key]['position'][1] - 2
            y[1] = self.quads[key]['position'][1] + 2
            self.ax.set_ylim3d(y)
            y[0] = 0
            y[1] = self.quads[key]['position'][2] + 5
            self.ax.set_zlim3d(y)
        plt.pause(0.000000000000001)

    def keypress_routine(self,event):
        sys.stdout.flush()
        if event.key == 'x':
            y = list(self.ax.get_ylim3d())
            y[0] += 0.2
            y[1] += 0.2
            self.ax.set_ylim3d(y)
        elif event.key == 'w':
            y = list(self.ax.get_ylim3d())
            y[0] -= 0.2
            y[1] -= 0.2
            self.ax.set_ylim3d(y)
        elif event.key == 'd':
            x = list(self.ax.get_xlim3d())
            x[0] += 0.2
            x[1] += 0.2
            self.ax.set_xlim3d(x)
        elif event.key == 'a':
            x = list(self.ax.get_xlim3d())
            x[0] -= 0.2
            x[1] -= 0.2
            self.ax.set_xlim3d(x)
    
    def plot_opaque_cube(self, x, y, z, dx, dy, dz):
        xx = [x, x, x+dx, x+dx, x]
        yy = [y, y+dy, y+dy, y, y]
        kwargs = {'alpha': 1, 'color': 'orange'}
        self.ax.plot3D(xx, yy, [z], **kwargs)
        self.ax.plot3D(xx, yy, [z+dz], **kwargs)
        self.ax.plot3D([x, x], [y, y], [z, z+dz], **kwargs)
        self.ax.plot3D([x, x], [y+dy, y+dy], [z, z+dz], **kwargs)
        self.ax.plot3D([x+dx, x+dx], [y+dy, y+dy], [z, z+dz], **kwargs)
        self.ax.plot3D([x+dx, x+dx], [y, y], [z, z+dz], **kwargs)

        bot = [(x, y, z),
               (x + dx, y, z),
               (x + dx, y + dy, z),
               (x, y+dy, z)]
        
        top = [ (x, y, z + dz),
                (x + dx, y, z + dz),
                (x + dx, y + dy, z + dz),
                (x, y+dy, z + dz)]
        
        left = [(x, y, z),
                (x, y + dy, z),
                (x, y + dy, z + dz),
                (x, y, z + dz)]

        right = [(x + dx, y, z),
                 (x + dx, y + dy, z),
                 (x + dx, y + dy, z + dz),
                 (x + dx, y, z + dz)]

        front = [(x, y, z),
                 (x + dx, y, z),
                 (x + dx, y, z + dz),
                 (x, y, z + dz)]

        back =  [(x, y + dy, z),
                 (x + dx, y + dy, z),
                 (x + dx, y + dy, z + dz),
                 (x, y + dy, z + dz)]

        face1 = mp3d.art3d.Poly3DCollection([bot], alpha=0.3, linewidth=1)
        face2 = mp3d.art3d.Poly3DCollection([top], alpha=0.3, linewidth=1)
        face3 = mp3d.art3d.Poly3DCollection([left], alpha=0.3, linewidth=1)        
        face4 = mp3d.art3d.Poly3DCollection([right], alpha=0.3, linewidth=1)        
        face5 = mp3d.art3d.Poly3DCollection([front], alpha=0.3, linewidth=1)        
        face6 = mp3d.art3d.Poly3DCollection([back], alpha=0.3, linewidth=1)        

        # This is the key step to get transparency working
        alpha = 0.2
        face1.set_facecolor((0, 0, 1, alpha))
        face2.set_facecolor((0, 0, 1, alpha))
        face3.set_facecolor((0, 0, 1, alpha))
        face4.set_facecolor((0, 0, 1, alpha))
        face5.set_facecolor((0, 0, 1, alpha))
        face6.set_facecolor((0, 0, 1, alpha))

        self.ax.add_collection3d(face1)
        self.ax.add_collection3d(face2)
        self.ax.add_collection3d(face3)
        self.ax.add_collection3d(face4)
        self.ax.add_collection3d(face5)
        self.ax.add_collection3d(face6)