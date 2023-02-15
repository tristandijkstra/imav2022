from datetime import datetime
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from datetime import datetime
# import matplotlib
# matplotlib.use('Qt5Agg')

# Adapted from matplotlib docs:
# https://matplotlib.org/stable/gallery/misc/multiprocess_sgskip.html


class ProcessPlotter:
    def __init__(self, refreshTime, plottedProfiles, profilesDict):
        self.plottedProfiles = plottedProfiles
        self.profilesDict = profilesDict
        self.refreshTime = refreshTime
        self.P = pd.DataFrame()
        self.plotts = []

        # magic friend that ensures plots show up on correct subplot
        val, keys = list(profilesDict.values()), list(profilesDict.keys())
        lens = [len(u.keys()) for u in val]
        self.magic = [sum(lens[0:x]) for x in range(len(keys))]

    def terminate(self):
        self.P.to_csv(f"data/{datetime.now()}")
        plt.close("all")

    def callBack(self, i):
        while not self.pipe.empty():
            command = self.pipe.get()
            if command is None:
                self.terminate()
                return False
            else:
                timestamp, data = command

                temp = {
                    # remove the last ms to collate all profiles
                    "timestamp": [(timestamp // 10)],
                    # add the data stuff
                    **{key: [value] for key, value in data.items()},
                }

                self.P = (
                    pd.concat(
                        [self.P, pd.DataFrame.from_dict(temp).set_index("timestamp")],
                        axis=0,
                        ignore_index=False,
                    )
                    .groupby("timestamp")
                    .first()
                    .fillna(method="ffill")
                )

                ### TODO FIX TO ALLOW SINGLE PROFILES ALSO
                # print(self.P.tail())
                for idxProf, profile in enumerate(self.plottedProfiles):
                    try:
                        data = self.P.loc[:, list(self.profilesDict[profile].keys())]
                        cols = data.columns.values

                        mn, mx = data.min().min(), data.max().max()
                        self.ax[idxProf].set_xlim(
                            min(data.index.values), max(data.index.values)
                        )
                        for idx, col in enumerate(cols):
                            self.plotts[idx + self.magic[idxProf]].set_data(
                                data.index.values, data.loc[:, col].values
                            )
                            # self.ax.set_ylim(mn, mx)
                    except Exception as e:
                        print(f"Error in plotter: {e}")

        return self.plotts

    def initPlot(self):
        for profile in range(len(self.plottedProfiles)):
            self.plotts += [
                self.ax[profile].plot([], [], label=prop)[0]
                for prop in self.profilesDict[self.plottedProfiles[profile]]
            ]
        if isinstance(self.ax, np.ndarray):
            for ax in range(len(self.ax)):
                self.ax[ax].set_xlim(0, 10000)
                self.ax[ax].set_ylim(-1.2, 1.2)
                self.ax[ax].legend()
        else:
            self.ax.set_xlim(0, 10000)
            self.ax.set_ylim(-180, 180)
            self.ax.legend()
        return self.plotts

    def __call__(self, pipe):
        self.pipe = pipe
        self.fig, self.ax = plt.subplots(len(self.plottedProfiles), 1)

        self.ani = FuncAnimation(
            self.fig,
            self.callBack,
            init_func=self.initPlot,
            repeat=True,
            interval=self.refreshTime,
            blit=True,
        )

        print("Plotter Started")
        plt.show()
