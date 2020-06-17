import numpy as np
from abc import ABCMeta, abstractmethod
from scipy import signal

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
                                                        / self.wavelength + phase) #- 10./1000.
            z = self.center[2] + self.amplitude*np.cos(2*np.pi*(timestep + i)/ \
                                                        self.wavelength + phase) #- 10./1000.
            x = self.center[0]

            target[i, :] = [x,y,z]
            i+=1
        return target

class Pringle2:
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

        phase = self.find_projection_along_path(current_point)
        #phase = 0
        for i in range(n1, n2):
            y = self.amplitude / 2. * np.sin(2.*np.pi*(timestep+i) \
                                                        / (self.wavelength) + phase)
            z = self.amplitude * np.cos(2*np.pi*(timestep + i)/ \
                                            (self.wavelength) + phase) + 0./1000.
            x = self.amplitude * np.sin(100*z*y)

            target[i, :] = [self.center[0] + x, self.center[1] + y, self.center[2] + z]
        return target

class Pringle:
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

        phase = self.find_projection_along_path(current_point)
        #phase = 0
        i = 0

        for _ in range(n1, n2):
            z = self.amplitude * np.sin(2.*np.pi*(timestep + i) \
                                                        / self.wavelength + phase) + 0./1000.
            y = 0.0 #self.amplitude * np.sin(2*np.pi*(timestep + i)/ \
                #                                        self.wavelength + phase) + 0./1000.

            x = 0.0 #self.amplitude * np.sin(1000.*z*y) + 0./1000.

            target[i, :] = [self.center[0] + x, self.center[1] + y, self.center[2] + z]
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
    def __init__(self, frequency = 500, amplitude = 0.025, \
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

