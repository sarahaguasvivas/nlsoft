import numpy as np
from abc import ABCMeta, abstractmethod
from scipy import signal
from typing import List
from scipy import signal
from scipy.spatial.transform import Rotation as R

class Target(ABCMeta):
    def __init__(self):
        pass

    @abstractmethod
    def find_projection_along_path(self, current_point):
        pass

    @abstractmethod
    def spin(self, timestep, n1, n2, dims):
        pass

class Circle(Target):
    def __init__(self, wavelength = 500, amplitude = 0.025, \
                                center = [0.0, 0.0, 0.0]):
        self.wavelength = wavelength
        self.amplitude = amplitude
        self.center = center

    def find_projection_along_path(self, current_point):
        ang1 = np.arctan2(self.center[0], self.center[2])
        ang2 = np.arctan2(self.center[0], self.center[2])
        return (ang1 - ang2) % (2. * np.pi)

    def spin(self, timestep, n1, n2, dims, current_point):
        target = np.empty([n2 - n1, dims])
        #phase = self.find_projection_along_path(current_point)
        phase = 0
        i = 0
        for _ in range(n1, n2):
            y = self.center[1] + self.amplitude * np.sin(2.*np.pi*(timestep + i) \
                                                        / self.wavelength + phase) - 10./1000.
            z = self.center[2] + self.amplitude*np.cos(2*np.pi*(timestep + i)/ \
                                                        self.wavelength + phase) + 10./1000.
            x = self.center[0]

            target[i, :] = [x,y,z]
            i+=1
        return target

class Diagonal:
    def __init__(self, wavelength = 500, amplitude = 0.025, \
                                center = [0.0, 0.0, 0.0]):
        self.wavelength = wavelength
        self.amplitude = amplitude
        self.center = center

    def find_projection_along_path(self, current_point):
        ang1 = np.arctan2(self.center[0], self.center[2])
        ang2 = np.arctan2(self.center[0], self.center[2])
        return (ang1 - ang2) % (2. * np.pi)

    def spin(self, timestep, n1, n2, dims):
        target = np.empty([n2 - n1, dims])
        phase = 0
        for i in range(n1, n2):
            y = self.amplitude / 2. * np.sin(2.*np.pi*(timestep+ i ) \
                                            / (self.wavelength) + phase + \
                                                0.000001*(timestep + i )**2) - 5./1000.

            z = self.amplitude * np.sin(2*np.pi*(timestep + i  ) / \
                                            (self.wavelength) + phase +\
                                        0.000001*(timestep + i )**2)

            x = 0.0009*(y**2/(10./1000.)**2 - z**2 / (10./1000.)**2) - 1./1000.

            del_X = [x, y, z]
            rotation = np.array([0.3, 0.4 + np.pi, -0.3])
            rot = R.from_rotvec(rotation)
            del_X = rot.apply(del_X)

            target[i, :] = [del_X[0] + self.center[0],
                            del_X[1] + self.center[1],
                            del_X[2] + self.center[2]]
            #target[i, :] = [self.center[0] + x, self.center[1] + y, self.center[2] + z]
        return target

class Pringle:
    def __init__(self, wavelength = 500, amplitude = 0.025, \
                                center = [0.0, 0.0, 0.0]):
        self.wavelength = wavelength
        self.amplitude = amplitude
        self.center = center
        self.a = 100./1000.
        self.b = 100./1000.

    def find_projection_along_path(self):
        ang1 = np.arctan2(self.center[0], self.center[2])
        ang2 = np.arctan2(self.center[0], self.center[2])
        return (ang1 - ang2) % (2. * np.pi)

    def spin(self, timestep, n1, n2, dims, current_point = [0., 0., 0.]):
        target = np.empty([n2 - n1, dims])

        #phase = self.find_projection_along_path(current_point)
        phase = 0
        i = 0

        for _ in range(n1, n2):
            z = self.amplitude / 1.* np.sin(2.*np.pi*(timestep + i) \
                                                        / self.wavelength + phase) - 0./1000.
            y = self.amplitude / 2.* np.cos(2*np.pi*(timestep + i)/ \
                                                        self.wavelength + phase) - 0./1000.

            x = 0.003 *(z**2 / self.amplitude**2 - 4.* y**2 / self.amplitude**2) #45.* y * z + 0./1000.

            del_X = [x, y, z]
            rotation = np.array([-0.1 , -np.pi/4., np.pi/4.])
            rot = R.from_rotvec(rotation)
            del_X = rot.apply(del_X)

            target[i, :] = [del_X[0] + self.center[0],
                            del_X[1] + self.center[1],
                            del_X[2] + self.center[2]]

            i+=1
        return target

class SingleAxisSineWave:
    def __init__(self, wavelength = 500, amplitude = 0.025, \
                                center = [0.0, 0.0, 0.0], axis = 1):
        self.wavelength = wavelength
        self.amplitude = amplitude
        self.center = center
        self.axis = axis
        self.boundaries = [[-56,-43], \
                         [-61.7,-10.7], \
                         [-27.6, 12.3]]

    def find_projection_along_path(self, current_point):
        ang1 = np.arctan2(self.center[0], self.center[2])
        ang2 = np.arctan2(self.center[0], self.center[2])
        return (ang1 - ang2) % (2. * np.pi)

    def spin(self, timestep, n1, n2, dims, current_point):
        target = np.empty([n2 - n1, dims])
        phase = self.find_projection_along_path(current_point)
        i = 0
        for _ in range(n1, n2):
            target[i, :] = [self.center[0], self.center[1], self.center[2]]
            target[i, self.axis] += self.amplitude*np.sin(2*np.pi*(timestep + i) \
                                        / self.wavelength + phase)
            i+=1
        return target

class SingleAxisSquareWave:
    def __init__(self, frequency = 500, amplitude = 0.025, \
                                center = [0.0, 0.0, 0.0], axis = 1):
        self.frequency = frequency
        self.amplitude = amplitude
        self.center = center
        self.axis = axis
    def find_projection_along_path(self, current_point):
        ang1 = np.arctan2(self.center[0], self.center[2])
        ang2 = np.arctan2(self.center[0], self.center[2])
        return (ang1 - ang2) % (2. * np.pi)

    def spin(self, timestep, n1, n2, dims, current_point):
        target = np.empty([n2 - n1, dims])

        phase = self.find_projection_along_path(current_point)
        i = 0
        for _ in range(n1, n2):
            target[i, :] = [(self.amplitude * signal.square(2.*np.pi*(timestep + i) \
                                                        / self.frequency + phase))*float(self.axis == 0),
                            (self.amplitude * signal.square(2.*np.pi*(timestep + i) \
                                                        / self.frequency + phase))*float(self.axis == 1),
                            (self.amplitude*signal.square(2*np.pi*(timestep + i) \
                                                        / self.frequency + phase))*float(self.axis == 2)]
            i+=1
        return target

class Square3D:
    def __init__(self, frequency = 500, amplitude = 0.025,
                                center = [0.0, 0.0, 0.0]):
        self.frequency = frequency
        self.amplitude = amplitude
        self.center = center

    def find_projection_along_path(self, current_point):
        ang1 = np.arctan2(self.center[0], self.center[2])
        ang2 = np.arctan2(self.center[0], self.center[2])
        return (ang1 - ang2) % (2. * np.pi)

    def spin(self, timestep, n1, n2, dims, current_point):
        target = np.empty([n2 - n1, dims])

        phase = self.find_projection_along_path(current_point)
        i = 0
        for _ in range(n1, n2):
            target[i, :] = [self.center[0],
                            self.center[1] + self.amplitude * signal.square(2.*np.pi*(timestep + i) \
                                                        / self.frequency + phase),
                            self.center[2] + self.amplitude*signal.square(2.*np.pi*(timestep + i) \
                                                        / self.frequency + phase + np.pi / 2.0)]
            i+=1
        return target


class FigureEight:
    def __init__(self, a : float = 20. / 1000.,
                 b : float = 10./1000.,
                 center : List[float] = [0., 0., 0.],
                 wavelength : float = 1000.):
        self.a = a
        self.b = b
        self.center = center
        self.wavelength = wavelength

    def spin(self, timestep, n1, n2, dims):
        target = np.empty([n2-n1, dims])
        i = 0
        for _ in range(n1, n2):
            y = self.a * np.sin((timestep + i) / self.wavelength)  + 0./1000.
            z = self.b * np.sin((timestep + i) / self.wavelength) * \
               np.cos((timestep + i)/self.wavelength) - 0./1000.
            x = -0.002*np.sin((timestep+i) / (self.wavelength))**2 + 0./1000.

            del_X = [x, y, z]

            rotation = np.array([-0.0, -0.7, -0.1])
            rot = R.from_rotvec(rotation)
            del_X = rot.apply(del_X)

            target[i, :] = [del_X[0] + self.center[0] + 0./1000.,
                            del_X[1] + self.center[1],
                            del_X[2] + self.center[2] + 0./1000.]
            #target[i, :] = rot.apply(target[i, :])

            target[i, :] = [target[i, 0]  - 0./1000.,
                            target[i, 1]  - 0./1000.,
                            target[i, 2]  + 0./1000.]
            i+=1
        return target

class FixedTarget:
    def __init__(self, a : float = 20. / 1000.,
                 b : float = 10./1000.,
                 center : List[float] = [0., 0., 0.],
                 wavelength : float = 1000.):
        self.a = a
        self.b = b
        self.center = center
        self.wavelength = wavelength

    def spin(self, timestep, n1, n2, dims):
        target = np.empty([n2-n1, dims])
        i = 0
        for _ in range(n1, n2):
            z = self.a #self.a * np.sin((timestep + i) / self.wavelength) + 0./1000.
            y = self.b #self.b * np.sin((timestep + i) / self.wavelength) * \
                #np.cos((timestep + i)/self.wavelength) + 0./1000.
            x = 0.0 #-0.003 * np.cos(2.*(timestep + i) / self.wavelength) * \
                #np.sin(2.*(timestep + i)/(self.wavelength)) + 0./1000.
            target[i, :] = [x + self.center[0],
                            y + self.center[1],
                            z + self.center[2]]
            i+=1
        return target
