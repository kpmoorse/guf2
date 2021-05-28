import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def npread(file, sep=',', header=None):
    df = pd.read_csv(file, sep=sep, header=header)
    arr = np.asarray(df)
    return arr

def npwrite(arr, file):
    df = pd.DataFrame(arr)
    df.to_csv(file, header=False, index=False)

def mix(color1, color2, val):
	res = []
	for c1, c2 in zip(color1,color2):
		res.append(c1*(1-val)+c2*(val))
	return res

gray = (0.1,0.1,0,1)
blue = (0,0.5,0.9)
red = (0.9,0.2,0)

sa = []
res = []
mode = "my"

for j, folder in enumerate(['res_neg','res_pos']):

	f0 = npread('{}/{}_00.csv'.format(folder, mode))
	wb = np.arange(f0.shape[0])/100

	fp = []
	for i in range(2):
		fp.append(npread('{}/{}_p{}.csv'.format(folder,mode,i+1)))

	fn = []
	for i in range(2):
		fn.append(npread('{}/{}_n{}.csv'.format(folder,mode,i+1)))

	# print(np.mean(np.asarray(f0),axis=0))

	print(np.mean(np.asarray(f0)[:,0]))

	res.append(np.hstack(
		(np.mean(np.asarray(fn),axis=1).T[0][::-1],
		np.mean(np.asarray(f0),axis=0),
		np.mean(np.asarray(fp),axis=1).T[0])
		))


	plt.subplot(2,2,(2*j)+1)

	plt.plot(wb,f0, color=gray)
	for i, data in enumerate(fp):
		plt.plot(wb,data,'--' , color=mix(gray,blue,0.25*(i+2)))
	for i, data in enumerate(fn):
		plt.plot(wb,data,'--' , color=mix(gray,red,0.25*(i+2)))

	plt.gca().spines["right"].set_visible(False)
	plt.gca().spines["top"].set_visible(False)
	# plt.legend(["Hovering","Negative coeff","$+F_z$ mode"], loc=3)
	# plt.legend(["Hovering","_nolegend_","_nolegend_","$-F_z$ mode","_nolegend_","_nolegend_","$+F_z$ mode"], loc=3)
	plt.xlabel("Time (wingbeats)")
	plt.ylabel("{} ({})".format(folder, mode))
	# plt.ylim([-15,30])

plt.subplot(1,2,2)
plt.plot(np.arange(-2,3)*1e-5, res[0], '.-')
plt.plot(np.arange(-2,3)*1e-5, res[1], '.-')
plt.legend(["Negative mode","Positive mode"])
plt.xlabel("Mode strength")
# plt.ylabel("Wingbeat-averaged force")

plt.show()

# mag = np.arange(-5,6)
# stroke_avg = []
# for f in fp[::-1]:
# 	stroke_avg.append(np.mean(f))
# stroke_avg.append(np.mean(f0))
# for f in fn:
# 	stroke_avg.append(np.mean(f))

# # plt.plot(mag,stroke_avg)
# for i in range(6):
# 	plt.plot(mag[i:i+2],stroke_avg[i:i+2], '.-', lw=3, ms=10, color=mix(red,gray,i/5))
# for i in np.arange(6,11):
# 	plt.plot(mag[i:i+2],stroke_avg[i:i+2], '.-', lw=3, ms=10, color=mix(gray,blue,(i-5)/5))

# plt.xlabel("Control Magnitude")
# plt.ylabel("Stroke-Averaged $F_z$")
# plt.gca().spines["right"].set_visible(False)
# plt.gca().spines["top"].set_visible(False)
# plt.show()