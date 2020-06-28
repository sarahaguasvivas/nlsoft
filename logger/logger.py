import numpy as np
from scipy import stats

class Logger:
    def __init__(self):
        self.log_dictionary = {}

    def add_to_log(self, **kwargs):
        pass

    def log(self, log_dict):
        for key in log_dict.keys():
            if key not in self.log_dictionary.keys():
                self.log_dictionary[key] = log_dict[key]
            else:
                for logged in log_dict[key].keys():
                    if logged not in self.log_dictionary[key].keys():
                        self.log_dictionary[key][logged] = [log_dict[key][logged]]
                    else:
                        self.log_dictionary[key][logged] += [log_dict[key][logged]]

    def verbose(self, **kwargs):
        print "__________________________________"
        for key, value in kwargs.items():
            print "GPC : {0} = {1}".format(key, value)

    def save_log(self, filename = 'log_output.json'):
        import json
        with open(filename, 'w') as fp:
            json.dump(self.log_dictionary, fp)

    def plot_log(self):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d, Axes3D
        font = {'family': 'serif',
                'size': 14}

        plt.rcParams["font.family"] = "Times New Roman"

        plt.style.use('seaborn')

        ym = []
        yn = []
        predicted_ = []
        actual_ = []
        u_optimal_list = []
        elapsed = []
        signal = []

        # for all experiments:
        for key in self.log_dictionary.keys():
            if key != "metadata":
                ym += [self.log_dictionary[key]['ym']]
                yn += [self.log_dictionary[key]['yn']]
                actual_ += [self.log_dictionary[key]['actual']]
                u_optimal_list += [self.log_dictionary[key]['u']]
                elapsed+= [self.log_dictionary[key]['elapsed']]
                signal += [self.log_dictionary[key]['signal']]
        NUM_EXPERIMENTS=self.log_dictionary['metadata']['num_experiments']
        NUM_TIMESTEPS = self.log_dictionary['metadata']['num_timesteps']
        neutral_point = self.log_dictionary['metadata']['neutral_point']

        print "Average control loop time in seconds:  ", np.mean(np.abs(elapsed))

        ym = 1000*np.reshape(ym, (NUM_EXPERIMENTS,-1, 3))
        yn = 1000*np.reshape(yn, (NUM_EXPERIMENTS, -1, 3))
        actual_ = 1000*np.reshape(actual_, (NUM_EXPERIMENTS,-1, 3))
        elapsed_ = np.reshape(elapsed, (NUM_EXPERIMENTS, -1, 1))

        error_p=np.abs((yn - actual_) / actual_)
        error_e=np.abs((actual_ - ym) / ym)

        print "Average prediction error: ", np.mean(error_p, axis = (0, 1))
        print "Average control error: ", np.mean(error_e, axis = (0, 1))

        print "Standard error prediction: ", np.std(error_p, axis=(0, 1), ddof=0)
        print "Standard error control: ", np.std(error_e, axis=(0, 1), ddof=0)

        u_optimal_list = np.reshape(u_optimal_list, (NUM_EXPERIMENTS, -1, 2))
        error_mm = (yn - ym) / np.maximum(ym, 1)
        error_pred = (yn - actual_)/ np.maximum(actual_, 1)
        signal = np.reshape(signal, (NUM_EXPERIMENTS, -1, 11))

        # Zeroing each state:
        max_target = np.max(ym, axis = (0, 1))
        min_target = np.min(ym, axis = (0, 1))
        shift = (max_target + min_target) / 2.

        color_palette = ['#1446A0', '#DB3069', '#F5D547', '#F5D547', '#3C3C3B']
        labels = ['x', 'y', 'z', 'u']
        plt.figure()
        AXIS = 0
        timesteps = range(max(yn.shape))
        for i in range(1, 3, 1):
            plt.subplot(2, 1, i)
            plt.plot(np.mean(ym, axis = AXIS)[:, i] - shift[i], color = color_palette[-1], linestyle = 'dashed', label =r"$" + labels[i] + "_{target}$")
            plt.plot(np.mean(yn, axis = AXIS)[:, i] - shift[i], color_palette[0], label = r"$\hat{" + labels[i] + "}$" )
            plt.fill_between(timesteps, np.mean(yn, axis = AXIS)[:, i] - np.std(yn, axis = AXIS)[:, i] - shift[i],
                                np.mean(yn, axis = AXIS)[:, i] + np.std(yn, axis = AXIS)[:, i] - shift[i],
                                    color = color_palette[0], alpha = 0.5)
            plt.plot(np.mean(actual_, axis = AXIS)[:, i] - shift[i], color = color_palette[1], label =r"$" + labels[i] + "_{true}$") # only 0 and 2

            plt.fill_between(timesteps, np.mean(actual_, axis = AXIS)[:, i] - np.std(actual_, axis = AXIS)[:, i] - shift[i],
                                            np.mean(actual_, axis = AXIS)[:, i] + np.std(actual_, axis = AXIS)[:, i] - shift[i],
                                                color = color_palette[1], alpha = 0.5, label = r"$2\sigma$")

            plt.legend(prop = {"family": "Times New Roman", "size": 14})
            plt.ylabel(str(labels[i]) + ' [mm]', **font)
            plt.xticks(fontsize=14, **font)
            plt.yticks(fontsize=14, **font)
            plt.plot(1000*neutral_point[i]- shift[i], marker = 'h')
            if i==2:
                plt.xlabel('timesteps', **font)
            if i==0:
                plt.title("Changes in States with respect to Timesteps", **font)
        plt.show()

        max_input = np.max(np.max(u_optimal_list))
        min_input = np.min(np.min(u_optimal_list))
        color_palette1= ['#272838', '#F3DE8A', '#F3DE8A', '#F3DE8A']

        plt.figure()
        for i in range(2):
            plt.subplot(2, 1, i+1)
            plt.plot(np.mean(u_optimal_list, axis =AXIS)[:, i], color = color_palette1[0], label = r'$u_{' + str(i) + "}$" )
            plt.fill_between(timesteps, np.mean(u_optimal_list, axis = AXIS)[:, i] -  np.std(u_optimal_list, axis = AXIS)[:, i],\
                                            np.mean(u_optimal_list, axis = AXIS)[:, i] + np.std(u_optimal_list, axis = AXIS)[:, i],\
                                                color = color_palette1[0], alpha = 0.3, label = r"$2\sigma$")


            plt.legend(prop = {"family": "Times New Roman", "size": 14})
            plt.xticks(fontsize=14, **font)
            plt.yticks(fontsize=14, **font)
            #plt.ylim([min_input, max_input])
            plt.ylabel(str(labels[-1]) + ' [degrees]', **font)
            if i == 0:
                plt.title('Changes in Inputs with respect to Timesteps', **font)
        plt.xlabel('timesteps', **font)
        plt.show()

        #neutral_point = 1000*np.array(neutral_point).reshape(-1, 3)

        fig = plt.figure()

        m_predicted_ = np.mean(yn, axis = AXIS)
        m_ym = np.mean(ym, axis = AXIS)
        m_actual_ = np.mean(actual_, axis = AXIS)

        ax = Axes3D(fig)
        ax.plot3D(m_predicted_[:, 0], m_predicted_[:, 1], m_predicted_[:, 2],color = color_palette[0],  linewidth = 1, alpha = 0.9, label = 'estimated position')
        ax.plot3D(m_ym[:, 0], m_ym[:, 1], m_ym[:, 2], color = color_palette[-1],linestyle = 'dashed',  linewidth = 1, alpha = 1, label = 'target')
        ax.plot3D(m_actual_[:, 0], m_actual_[:, 1], m_actual_[:, 2], \
                            linewidth = 1, color = color_palette[1], alpha = 1, label = 'actual position')
        #ax.set_xlim(-20., 5.)
        #ax.set_ylim(-20., 5.)
        #ax.set_zlim(-20., 5.)
        plt.legend(prop = {"family": "Times New Roman", "size": 14})
        plt.xticks(fontsize=14, **font)
        plt.yticks(fontsize=14, **font)
        plt.xlabel('x[mm]', **font)
        plt.ylabel('y[mm]', **font)
        plt.title('Target Position vs. Controlled Positions', **font)
        plt.show()

        """
            Error plot
        """

        plt.figure()

        AXIS = 0
        timesteps = range(max(yn.shape))
        for i in range(3):
            plt.subplot(3, 1, i+1)
            plt.plot(np.mean(error_mm, axis = AXIS)[:, i], color = 'k', label = '% control error in ' + labels[i])
            plt.fill_between(timesteps, np.mean(error_mm, axis = AXIS)[:, i] - np.std(error_mm, axis = AXIS)[:, i] ,\
                                np.mean(error_mm, axis = AXIS)[:, i] + np.std(error_mm, axis = AXIS)[:, i], \
                                    color = 'k', alpha = 0.5)
            plt.plot(np.mean(error_pred, axis = AXIS)[:, i], color = 'gray', label = '%prediction error')
            plt.fill_between(timesteps, np.mean(error_pred, axis = AXIS)[:, i] - np.std(error_pred, axis = AXIS)[:, i] ,\
                                            np.mean(error_pred, axis = AXIS)[:, i] + np.std(error_pred, axis = AXIS)[:, i], \
                                                color = 'gray', alpha = 0.5)


            #plt.ylim([-50, 50])
            plt.legend(prop = {"family": "Times New Roman", "size": 14})
            plt.xticks(fontsize=14, **font)
            plt.yticks(fontsize=14, **font)
            plt.ylabel(str(labels[i]) + ' %', **font)
            if i==2:
                plt.xlabel('timesteps', **font)
            if i==0:
                plt.title("Off-track Error", **font)
        plt.show()
        """
            Elapsed Time Plot
        """
        plt.figure()
        AXIS = 0
        timesteps = range(max(elapsed_.shape))
        plt.plot(np.mean(elapsed_, axis = AXIS), color = '#D2691E', label = 'time [s]')
        plt.fill_between(timesteps, np.mean(elapsed_, axis = AXIS)[:, 0] - np.std(elapsed_,
            axis = AXIS)[:, 0] ,np.mean(elapsed_, axis = AXIS)[:, 0] + np.std(elapsed_, axis = AXIS)[:, 0],
                                color = '#D2691E', alpha = 0.5)
        plt.legend(prop = {"family": "Times New Roman", "size": 14})
        plt.ylabel('Time [s]', **font)
        plt.xlabel('timesteps', **font)
        plt.title("Control Loop Computation Time", **font)
        plt.ylim([0, 0.2])
        plt.xticks(fontsize=14, **font)
        plt.yticks(fontsize=14, **font)
        plt.show()

        plt.figure()
        plt.plot(np.mean(signal, axis = 0)[:-1, :])
        plt.title("Tracking of the Mean Optical Lace Signals During Experiment", **font)
        plt.xticks(fontsize=14, **font)
        plt.yticks(fontsize=14, **font)
        plt.show()

