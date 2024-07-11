import numpy as np

def forwardKinematics(angles):
    x = (-np.cos(np.radians(angles[0]))*((230*np.sin(np.radians(angles[1])))+(230*np.sin(np.radians(angles[1]+angles[2])))+(124.5*np.sin(np.radians(angles[1]+angles[2]+angles[3])))))
    y = (-np.sin(np.radians(angles[0]))*((230*np.sin(np.radians(angles[1])))+(230*np.sin(np.radians(angles[1]+angles[2])))+(124.5*np.sin(np.radians(angles[1]+angles[2]+angles[3])))))
    z = (187+(230*np.cos(np.radians(angles[1])))+(230*np.cos(np.radians(angles[1]+angles[2])))+(124.5*np.cos(np.radians(angles[1]+angles[2]+angles[3]))))
    se = (-np.sin(np.radians(angles[1]+angles[2]+angles[3])))
    ce = np.cos(np.radians(angles[1]+angles[2]+angles[3]))
    e = np.degrees(np.arctan2(se,ce))
    if(np.sin(np.radians(e))>=1e-6):
        a = angles[0]
        sr = (np.sin(np.radians(angles[4]))*np.sin(np.radians(angles[1]+angles[2]+angles[3]))/np.sin(np.radians(e)))
        cr = (-np.cos(np.radians(angles[4]))*np.sin(np.radians(angles[1]+angles[2]+angles[3]))/np.sin(np.radians(e)))
        r = np.degrees(np.arctan2(sr,cr))
    else:
        a = 0
        r = (angles[0]+(angles[4]*np.cos(np.radians(angles[1]+angles[2]+angles[3]))))
        if(r>180):
            r = (180-r)
        if(r< -180):
            r = (r+180)
        
    return np.array([round(x,3), round(y,3), round(z,3), round(a,3), round(e,3), round(r,3)])

def inverseKinematics(pose):
    #Elementos da matriz de rotação:
    e = pose[3]
    r = pose[4]
    if(np.sin(np.radians(e))>=1e-6):
        a = np.degrees(np.arctan2(pose[1], pose[0]))
    else:
        a = 0
    r11 = ((np.cos(np.radians(a))*np.cos(np.radians(e))*np.cos(np.radians(r)))-(np.sin(np.radians(a))*np.sin(np.radians(r))))
    r12 = (-(np.cos(np.radians(a))*np.cos(np.radians(e))*np.sin(np.radians(r)))-(np.sin(np.radians(a))*np.cos(np.radians(r))))
    r13 = (np.cos(np.radians(a))*np.sin(np.radians(e)))
    r21 = ((np.sin(np.radians(a))*np.cos(np.radians(e))*np.cos(np.radians(r)))+(np.cos(np.radians(a))*np.sin(np.radians(r))))
    r22 = (-(np.sin(np.radians(a))*np.cos(np.radians(e))*np.sin(np.radians(r)))+(np.cos(np.radians(a))*np.cos(np.radians(r))))
    r23 = (np.sin(np.radians(a))*np.sin(np.radians(e)))
    r31 = (-np.sin(np.radians(e))*np.cos(np.radians(r)))
    r32 = (-np.sin(np.radians(e))*np.sin(np.radians(r)))
    r33 = np.cos(np.radians(e))

    #Desacoplamento cinemático:
    xc = (pose[0]-(124.5*r13))
    yc = (pose[1]-(124.5*r23))
    zc = (pose[2]-(124.5*r33))

    #Vetor para armazenar as soluções:
    sol = np.empty((4, 5))

    #Theta 1:
    #Front:
    th = np.degrees(np.arctan2(yc, xc))
    sol[0][0] = th
    sol[1][0] = th
    #Back:
    if(th>0):
        sol[2][0] = th-180
        sol[3][0] = th-180
    else:
        sol[2][0] = th+180
        sol[3][0] = th+180

    #Theta 3:
    r = np.sqrt((xc**2)+(yc**2))
    h = np.sqrt((r**2)+((zc-187)**2))
    c3 = (((h**2)/(2*(230**2)))-1)
    s3 = np.sqrt(1-(c3**2))
    #Front Elbow Up:
    sol[0][2] = np.degrees(np.arctan2(-s3, c3))
    #Front Elbow Down:
    sol[1][2] = np.degrees(np.arctan2(s3, c3))
    #Back Elbow Up:
    sol[2][2] = np.degrees(np.arctan2(s3, c3))
    #Back Elbow Down:
    sol[3][2] = np.degrees(np.arctan2(-s3, c3))

    #Theta 2:
    beta = np.degrees(np.arctan2((zc-187), r))
    for i in range(len(sol)):
        if(i<2):
            alpha = np.degrees(np.arctan2((-230*np.sin(np.radians(sol[i][2]))), (230*(1+np.cos(np.radians(sol[i][2]))))))
            sol[i][1] = (alpha+beta-90)
        else:
            alpha = np.degrees(np.arctan2((230*np.sin(np.radians(sol[i][2]))), (230*(1+np.cos(np.radians(sol[i][2]))))))
            sol[i][1] = (90-alpha-beta)
        

    #Theta 4:
    for i in range(len(sol)):
        s4 = (-(r13*np.cos(np.radians(sol[i][0]))*np.cos(np.radians(sol[i][1]+sol[i][2])))-(r23*np.sin(np.radians(sol[i][0]))*np.cos(np.radians(sol[i][1]+sol[i][2])))-(r33*np.sin(np.radians(sol[i][1]+sol[i][2]))))
        c4 = (-(r13*np.cos(np.radians(sol[i][0]))*np.sin(np.radians(sol[i][1]+sol[i][2])))-(r23*np.sin(np.radians(sol[i][0]))*np.sin(np.radians(sol[i][1]+sol[i][2])))+(r33*np.cos(np.radians(sol[i][1]+sol[i][2]))))
        sol[i][3] = np.degrees(np.arctan2(s4, c4))

    #Theta 5:
    for i in range(len(sol)):
        if(a >= 1e-3):
            s5 = (-(r11*np.sin(np.radians(sol[i][0])))+(r21*np.cos(np.radians(sol[i][0]))))
            c5 = (-(r12*np.sin(np.radians(sol[i][0])))+(r22*np.cos(np.radians(sol[i][0]))))
            sol[i][4] = np.degrees(np.arctan2(s5, c5))
        else:
            sol[i][4] = ((pose[4]-sol[i][0])/np.cos(np.radians(sol[i][1]+sol[i][2]+sol[i][3])))
            #print(sol[i][4])
            if(sol[i][4]>180):
                sol[i][4] = (180-sol[i][4])
            if(sol[i][4]< -180):
                sol[i][4] = (sol[i][4]+180)

    return sol.round(4)