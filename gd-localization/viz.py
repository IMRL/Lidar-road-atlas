import numpy as np
from matplotlib import pyplot as plt

data = []
f = open('log.txt')
for line in f.readlines():
    data.append([float(it) for it in line.split(' ')])
f.close()

data = np.array(data)

# debug = 0
# data[:,1].sum()
print("tanslation RMSE(m): %.4f" % np.sqrt((data[:,1]*data[:,1]).sum() / data.shape[0]))
print("rotation RMSE(deg): %.4f" % (np.sqrt((data[:,2]*data[:,2]).sum() / data.shape[0]) * 180.0 / np.pi))
print("average time cost: %.4f" % (data[:,3].sum() / data.shape[0]))
print("average icp steps: %.4f" % (data[:,4].sum() / data.shape[0]))

plt.xlim((0, data.shape[0]))
plt.ylim((0, 5))
plt.plot(data[:,1])
# plt.plot([0, data.shape[0]], [0.1, 0.1])
plt.savefig('viz.png')