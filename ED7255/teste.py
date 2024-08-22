#Importando as bibliotecas:
import ed7255 as ed
import numpy as np
import matplotlib.pyplot as plt
import scienceplots

#Exibição dos gráficos em janela externa:
#%matplotlib qt

# Configurações de plot:
plt.style.use([
    'grid',
    'retro'
])
plt.rcParams['lines.linewidth'] = 2
plt.rcParams['font.size'] = 16 
plt.rcParams['figure.figsize'] = (12, 6)

#Declaração do objeto do robô:
robo = ed.ED7255()

#Iniciando a amostragem:
robo.initSampling()

#Movimentação livre joint:
robo.moveJoint([170, -10, -20, -30, 50])
#robo.moveJoint([-20, -30, -80, -15, 45])

# #Movimentação livre cartesiana:
# robo.move([300, 300, 200, 90, 0])
# robo.move([584.5, 0, 187, 90, 0])

# # #Movimento linear:
# robo.moveLinear([500, 0, 187, 90, 0])
# robo.moveLinear([460, 0, 187, 90, 0])

# #Movimento incremental via base-jog:
# robo.baseJog([0, 0, 13, -30, 0])

# #Movimento incremental via tool-jog:
# robo.move([0, 500, 200, 90, 0])
# robo.toolJog([0, 100, -100, 0, 0])

# #Demonstração do movelinear suavizado:
# robo.move([400, 400, 187, 90, 0])
# robo.initSampling()
# #robo.move([0, 500, 187, 90, 0])
# robo.moveLinear([400, -400, 187, 90, 0])

#Plotando as curvas:
#robo.plotData()



#Curvas para a monografia:

#Tempo e posição da junta 1:
tth = np.array(robo.getData()[:2])

#Vetor de tempo:
dtm = (tth[0][-1]-tth[0][-2])
t = np.append(tth[0], [(tth[0][-1]+(0.4*dtm)), (tth[0][-1]+(0.8*dtm))])

#Ângulo da junta 1:
th = np.append(tth[1], [tth[1][-1], tth[1][-1]])

#Velocidade da junta 1:
dth = np.array([0])
dth = np.append(dth, (th[1:]-th[:-1])/(t[1:]-t[:-1]))

#Aceleração da junta 1:
ddth = np.array([0])
ddth = np.append(ddth, (dth[1:]-dth[:-1])/(t[1:]-t[:-1]))

plt.subplots(3,1, sharex=True)
#plt.subplots_adjust(hspace=0.05)
plt.subplot(3, 1, 1)
plt.plot(t, th, color='C1', label='$q(t)$')
plt.ylabel('Posição')
plt.yticks([th[0], th[-1]], ['$q_0$', '$q_f$'])
plt.legend(loc='lower right')
plt.subplot(3, 1, 2)
plt.plot(t, dth, color='C0', label='$\dot{q}(t)$')
plt.ylabel('Velocidade')
plt.yticks([dth[0], dth[int(len(dth)/2)]], ['$0$', '$\dot{q}_{max}$'])
plt.legend(loc='center right')
plt.subplot(3, 1, 3)
plt.plot(t, ddth, color='C4', label='$\ddot{q}(t)$')
plt.ylabel('Aceleração')
plt.yticks([dth[0], ddth[int(len(ddth)/4)]+8e-7, ddth[int(3*len(ddth)/4)]-6e-7], ['$0$', '$\ddot{q}_{max}$', '$-\ddot{q}_{max}$'])
plt.xlabel('Tempo')
plt.xticks([0, t[int(len(t)/2)], t[-1]], ['$t_0$', '$\\frac{t_f-t_0}{2}$', '$t_f$'])
plt.legend(loc='upper right')
#plt.show()
plt.savefig('traj.eps', dpi=600, transparent=True, bbox_inches='tight')