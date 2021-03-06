from python.target.target import Target
import scipy.io as sio
import numpy as np

class VanisSwirl():
    def __init__(self, source_filename = '../models/ref_data.mat', neutral_point = [0]*3,
                            wavelength = 10):
        self.source_file = sio.loadmat(source_filename)
        self.reference_data = np.array(self.source_file['ref_data'])
        self.neutral_point = neutral_point
        self.wavelength = wavelength

    def find_projection_along_path(self, current_point):
        pass

    def spin(self, timestep, n1, n2, dims):
        i = 0
        target = np.empty([n2 - n1, dims])
        for _ in range(n1, n2):
            x = self.reference_data[(timestep + i)//self.wavelength, 0] + self.neutral_point[0]
            y = self.reference_data[(timestep + i)//self.wavelength, 1] + self.neutral_point[1]
            z = self.reference_data[(timestep + i)//self.wavelength, 2] + self.neutral_point[2]
            target[i, :] = [x,y,z]
            i+=1
        return target

if __name__ == "__main__":
    target = VanisSwirl()