import matplotlib.pyplot as plt
import numpy as np

out_file = np.loadtxt('out.txt')


x_out = out_file[:,0]
y_out = out_file[:,1] 


print(len(x_out))
print(len(y_out))
plt.plot(x_out, y_out, label='loc UTM')


plt.axes().set_aspect("equal","datalim")
plt.figure(1)
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.show()
