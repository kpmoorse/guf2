import numpy as np
import matplotlib.pyplot as plt

def cL(aoa):
	return 1.597 * np.sin(0.0407*aoa - 0.369) + 0.317

def cD(aoa):
	return 1.464 * np.sin(0.0342*aoa - 1.667) + 2.008

a = np.linspace(0,90,100)
ad = np.linspace(5,85,3)

plt.plot(ad,cL(ad),'b.',ad,cD(ad),'r.',markersize=15)
plt.plot(a,cL(a),'b',a,cD(a),'r',linewidth=3)

plt.xlim([0,90])
plt.ylim([-0.5,4])

plt.xlabel("Angle of attack (degrees)")
plt.ylabel("Force coefficients")

ax = plt.gca()
ax.spines["right"].set_visible(False)
ax.spines["top"].set_visible(False)
for spine in (ax.spines["left"], ax.spines["bottom"]):
	spine.set_position(["outward",5])
ax.spines["left"].set_bounds(plt.yticks()[0][0], plt.yticks()[0][-1])
ax.spines["bottom"].set_bounds(plt.xticks()[0][0], plt.xticks()[0][-1])

plt.show()