import numpy as np

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

        plt.style.use('seaborn')

        ym = []
        yn = []
        predicted_ = []
        actual_ = []
        u_optimal_list = []

        # for all experiments:
        for key in self.log_dictionary.keys():
            if key != "metadata":
                ym += [self.log_dictionary[key]['ym']]
                yn += [self.log_dictionary[key]['yn']]
                actual_ += [self.log_dictionary[key]['actual']]
                u_optimal_list += [self.log_dictionary[key]['u']]

        print self.log_dictionary['metadata']
        NUM_EXPERIMENTS=self.log_dictionary['metadata']['num_experiments']
        NUM_TIMESTEPS = self.log_dictionary['metadata']['num_timesteps']
        neutral_point = self.log_dictionary['metadata']['neutral_point']

        ym = 1000*np.reshape(ym, (NUM_EXPERIMENTS,-1, 3))
        yn = 1000*np.reshape(yn, (NUM_EXPERIMENTS, -1, 3))
        predicted_ = 1000*np.reshape(predicted_, (NUM_EXPERIMENTS, -1, 3))
        actual_ = 1000*np.reshape(actual_, (NUM_EXPERIMENTS,-1, 3))
        u_optimal_list = np.reshape(u_optimal_list, (NUM_EXPERIMENTS, -1, 2))

        labels = ['x', 'y', 'z', 'u']
        plt.figure()
        AXIS = 0
        timesteps = range(max(yn.shape))
        for i in range(3):
            plt.subplot(3, 1, i+1)
            plt.plot(np.mean(ym, axis = AXIS)[:, i], color = '#d3d3d3', linestyle = 'dashed', label = 'target')
            plt.plot(np.mean(yn, axis = AXIS)[:, i], '#bfcbc5', label = 'predicted state')
            plt.fill_between(timesteps, np.mean(yn, axis = AXIS)[:, i] - np.std(yn, axis = AXIS)[:, i] ,\
                                np.mean(yn, axis = AXIS)[:, i] + np.std(yn, axis = AXIS)[:, i], \
                                    color = '#bfcbc5', alpha = 0.5)
            plt.plot(np.mean(actual_, axis = AXIS)[:, i], color = 'goldenrod', label = 'actual state') # only 0 and 2

            plt.fill_between(timesteps, np.mean(actual_, axis = AXIS)[:, i] - np.std(actual_, axis = AXIS)[:, i],\
                                            np.mean(actual_, axis = AXIS)[:, i] + np.std(actual_, axis = AXIS)[:, i],\
                                                color = 'goldenrod', alpha = 0.5)

            plt.ylim([-0.1*1000, 0.09*1000])
            plt.legend()
            plt.ylabel(str(labels[i]) + ' [mm]')
            plt.plot(1000*neutral_point[i], marker = 'h')
            if i==2:
                plt.xlabel('timesteps')
            if i==0:
                plt.title("Changes in States with respect to Timesteps")
        plt.show()

        max_input = np.max(np.max(u_optimal_list))
        min_input = np.min(np.min(u_optimal_list))
        plt.figure()
        for i in range(2):
            plt.subplot(2, 1, i+1)
            plt.plot(np.mean(u_optimal_list, axis =AXIS)[:, i], color = 'slateblue', label = r'$u_{' + str(i) + "}$" )
            plt.fill_between(timesteps, np.mean(u_optimal_list, axis = AXIS)[:, i] -  np.std(u_optimal_list, axis = AXIS)[:, i],\
                                            np.mean(u_optimal_list, axis = AXIS)[:, i] + np.std(u_optimal_list, axis = AXIS)[:, i],\
                                                color = 'slateblue', alpha = 0.3, label = r"$2\sigma$")


            plt.legend()
            #plt.ylim([min_input, max_input])
            plt.ylabel(str(labels[-1]) + ' [degrees]')
            if i == 0:
                plt.title('Changes in Inputs with respect to Timesteps')
        plt.xlabel('timesteps')
        plt.show()

        neutral_point = 1000*np.array(neutral_point).reshape(-1, 3)

        fig = plt.figure()
        m_predicted_ = np.mean(predicted_, axis = AXIS)
        m_ym = np.mean(ym, axis = AXIS)
        m_actual_ = np.mean(actual_, axis = AXIS)
        ax = Axes3D(fig)
        ax.plot3D(m_predicted_[:, 0],m_predicted_[:, 1], m_predicted_[:, 2],color = 'c',  linewidth = 1, alpha = 0.9, label = 'estimated position')
        ax.plot3D(m_ym[:, 0], m_ym[:, 1], m_ym[:, 2], color = 'grey',linestyle = 'dashed',  linewidth = 1, alpha = 1, label = 'target')
        ax.plot3D(m_actual_[:, 0], m_actual_[:, 1], m_actual_[:, 2], \
                            linewidth = 1, color = 'goldenrod', alpha = 1, label = 'actual position')
        ax.set_xlim(-0.1*1000, .1*1000)
        ax.set_ylim(-.1*1000, .1*1000)
        ax.set_zlim(-.1*1000, .1*1000)
        plt.legend()
        plt.xlabel('x[mm]')
        plt.ylabel('y[mm]')
        plt.title('Target Position vs. Controlled Positions')
        plt.show()
