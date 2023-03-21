# import sys
# import lcm
# # from exlcm import example_t
# if len(sys.argv) < 2:
#     sys.stderr.write("usage: read-log <logfile>\n")
#     sys.exit(1)
# log = lcm.EventLog(sys.argv[1], "r")
# for event in log:
#     print(event.channel)
import numpy as np
import matplotlib.pyplot as plt

m1 = np.loadtxt("../test_logs/1.1test_concrete_both_dir_m1_new.txt",skiprows = 1)
m3 = np.loadtxt("../test_logs/1.1test_concrete_both_dir_m3_new.txt",skiprows = 1)
print(m1.shape)
print(m3.shape)


# fit 1d function for pwm(velocity)
slope_l_p, intercept_l_p = np.polyfit(m1[:,1][m1[:,1]>0], m1[:,0][m1[:,1]>0], 1)
slope_l_n, intercept_l_n = np.polyfit(m1[:,1][m1[:,1]<0], m1[:,0][m1[:,1]<0], 1)
slope_r_p, intercept_r_p = np.polyfit(m3[:,1][m3[:,1]>0], m3[:,0][m3[:,1]>0], 1)
slope_r_n, intercept_r_n = np.polyfit(m3[:,1][m3[:,1]<0], m3[:,0][m3[:,1]<0], 1)
l_p = np.poly1d([slope_l_p, intercept_l_p]) 
l_n = np.poly1d([slope_l_n, intercept_l_n]) 
r_p = np.poly1d([slope_r_p, intercept_r_p]) 
r_n = np.poly1d([slope_r_n, intercept_r_n]) 


fig = plt.figure() 
ax1 = plt.subplot(2,2,1); ax2 = plt.subplot(2,2,2)
ax1.plot(m1[:,1][m1[:,1]>0],m1[:,0][m1[:,1]>0],'ro')
ax1.plot(m1[:,1][m1[:,1]>0],l_p(m1[:,1][m1[:,1]>0]))
ax1.set_ylabel("duty factor")
ax1.set_xlabel("velocity")
ax1.set_title("motor 1, left, positive")
ax1.legend(["raw","fitted"])
ax1.grid()

ax2.plot(m1[:,1][m1[:,1]<0],m1[:,0][m1[:,1]<0],'ro')
ax2.plot(m1[:,1][m1[:,1]<0],l_n(m1[:,1][m1[:,1]<0]))
ax2.set_ylabel("duty factor")
ax2.set_xlabel("velocity")
ax2.set_title("motor 1, left, negative")
ax2.legend(["raw","fitted"])
ax2.grid()

ax3 = plt.subplot(2,2,3); ax4 = plt.subplot(2,2,4)
ax3.plot(m3[:,1][m3[:,1]>0],m3[:,0][m3[:,1]>0],'ro')
ax3.plot(m3[:,1][m3[:,1]>0],r_p(m3[:,1][m3[:,1]>0]))
ax3.set_ylabel("duty factor")
ax3.set_xlabel("velocity")
ax3.set_title("motor 3, right, positive")
ax3.legend(["raw","fitted"])
ax3.grid()

ax4.plot(m3[:,1][m3[:,1]<0],m3[:,0][m3[:,1]<0],'ro')
ax4.plot(m3[:,1][m3[:,1]<0],r_n(m3[:,1][m3[:,1]<0]))
ax4.set_ylabel("duty factor")
ax4.set_xlabel("velocity")
ax4.set_title("motor 3, right, negative")
ax4.grid()
ax4.legend(["raw","fitted"])

print("slope and intercept")
print('left positive velocity:', slope_l_p, intercept_l_p)
print('left negative velocity:', slope_l_n, intercept_l_n)
print('right positive velocity:', slope_r_p, intercept_r_p)
print('right negative velocity:',slope_r_n, intercept_r_n)

plt.show()
plt.savefig('motor_calibration.png', bbox_inches='tight', dpi = 600)
