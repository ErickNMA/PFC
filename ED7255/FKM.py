from sympy import *
import numpy as np

init_printing()

def arredNUM(matrix, n=3):
    for a in preorder_traversal(matrix):
        if isinstance(a, Float):
            matrix = matrix.subs(a, round(a, n))
    return matrix

def A(tn, dn, an, aln):
    M = Matrix([
            [ cos(tn), -sin(tn)*round(cos(aln),3), sin(tn)*round(sin(aln),3), an*cos(tn) ], 
            [ sin(tn), cos(tn)*round(cos(aln),3), -cos(tn)*round(sin(aln),3), an*sin(tn) ],
            [ 0, round(sin(aln),3), round(cos(aln),3), dn ],
            [0, 0, 0, 1]
            ])
    
    M.simplify()
    M = trigsimp(M)
    M = arredNUM(M)
    return M

#Thetas do diagrama de arrames
theta1 = Symbol('θ₁')
theta2 = Symbol('θ₂')
theta3 = Symbol('θ₃')
theta4 = Symbol('θ₄')
theta5 = Symbol('θ₅')
theta6 = Symbol('θ₆')

thetas = [theta1, theta2, theta3, theta4, theta5, theta6]

#Deslocamentos
d1 = Symbol('d₁')
d2 = Symbol('d₂')
d3 = Symbol('d₃') 
d4 = Symbol('d₄')
d5 = Symbol('d₅')
d6 = Symbol('d₆')

ds = [d1, d2, d3, d4, d5, d6]

#Tamanho dos links do manipulador
l1 = Symbol('l₁')
l2 = Symbol('l₂')
l3 = Symbol('l₃') 
l4 = Symbol('l₄')
l5 = Symbol('l₅')
l6 = Symbol('l₆')

ls = [l1, l2, l3, l4, l5, l6]

def abrev(M):
    O = M.subs(cos(theta1+theta2+theta3), Symbol('c123'))
    O = O.subs(cos(theta1+theta2+theta4), Symbol('c124'))
    O = O.subs(cos(theta1+theta2+theta5), Symbol('c125'))
    O = O.subs(cos(theta1+theta2+theta6), Symbol('c126'))
    O = O.subs(cos(theta1+theta3+theta4), Symbol('c134'))
    O = O.subs(cos(theta1+theta3+theta5), Symbol('c135'))
    O = O.subs(cos(theta1+theta3+theta6), Symbol('c136'))
    O = O.subs(cos(theta1+theta4+theta5), Symbol('c145'))
    O = O.subs(cos(theta1+theta4+theta6), Symbol('c146'))
    O = O.subs(cos(theta1+theta5+theta6), Symbol('c156'))
    O = O.subs(cos(theta2+theta3+theta4), Symbol('c234'))
    O = O.subs(cos(theta2+theta3+theta5), Symbol('c235'))
    O = O.subs(cos(theta2+theta3+theta6), Symbol('c236'))
    O = O.subs(cos(theta3+theta4+theta5), Symbol('c345'))
    O = O.subs(cos(theta3+theta4+theta6), Symbol('c346'))
    O = O.subs(cos(theta4+theta5+theta6), Symbol('c456'))
    O = O.subs(cos(theta1-theta2), Symbol('c_{(1-2)}'))
    O = O.subs(cos(theta1-theta3), Symbol('c_{(1-3)}'))
    O = O.subs(cos(theta1-theta4), Symbol('c_{(1-4)}'))
    O = O.subs(cos(theta1-theta5), Symbol('c_{(1-5)}'))
    O = O.subs(cos(theta1-theta6), Symbol('c_{(1-6)}'))
    O = O.subs(cos(theta2-theta3), Symbol('c_{(2-3)}'))
    O = O.subs(cos(theta2-theta4), Symbol('c_{(2-4)}'))
    O = O.subs(cos(theta2-theta5), Symbol('c_{(2-5)}'))
    O = O.subs(cos(theta2-theta6), Symbol('c_{(2-6)}'))
    O = O.subs(cos(theta3-theta4), Symbol('c_{(3-4)}'))
    O = O.subs(cos(theta3-theta5), Symbol('c_{(3-5)}'))
    O = O.subs(cos(theta3-theta6), Symbol('c_{(3-6)}'))
    O = O.subs(cos(theta4-theta5), Symbol('c_{(4-5)}'))
    O = O.subs(cos(theta4-theta6), Symbol('c_{(4-6)}'))
    O = O.subs(cos(theta5-theta6), Symbol('c_{(5-6)}'))
    O = O.subs(cos(theta1+theta2), Symbol('c12'))
    O = O.subs(cos(theta1+theta3), Symbol('c13'))
    O = O.subs(cos(theta1+theta4), Symbol('c14'))
    O = O.subs(cos(theta1+theta5), Symbol('c15'))
    O = O.subs(cos(theta1+theta6), Symbol('c16'))
    O = O.subs(cos(theta2+theta3), Symbol('c23'))
    O = O.subs(cos(theta2+theta4), Symbol('c24'))
    O = O.subs(cos(theta2+theta5), Symbol('c25'))
    O = O.subs(cos(theta2+theta6), Symbol('c26'))
    O = O.subs(cos(theta3+theta4), Symbol('c34'))
    O = O.subs(cos(theta3+theta5), Symbol('c35'))
    O = O.subs(cos(theta3+theta6), Symbol('c36'))
    O = O.subs(cos(theta4+theta5), Symbol('c45'))
    O = O.subs(cos(theta4+theta6), Symbol('c46'))
    O = O.subs(cos(theta5+theta6), Symbol('c56'))
    O = O.subs(cos(theta1), Symbol('c1'))
    O = O.subs(cos(theta2), Symbol('c2'))
    O = O.subs(cos(theta3), Symbol('c3'))
    O = O.subs(cos(theta4), Symbol('c4'))
    O = O.subs(cos(theta5), Symbol('c5'))
    O = O.subs(cos(theta6), Symbol('c6'))

    O = O.subs(sin(theta1+theta2+theta3), Symbol('s123'))
    O = O.subs(sin(theta1+theta2+theta4), Symbol('s124'))
    O = O.subs(sin(theta1+theta2+theta5), Symbol('s125'))
    O = O.subs(sin(theta1+theta2+theta6), Symbol('s126'))
    O = O.subs(sin(theta1+theta3+theta4), Symbol('s134'))
    O = O.subs(sin(theta1+theta3+theta5), Symbol('s135'))
    O = O.subs(sin(theta1+theta3+theta6), Symbol('s136'))
    O = O.subs(sin(theta1+theta4+theta5), Symbol('s145'))
    O = O.subs(sin(theta1+theta4+theta6), Symbol('s146'))
    O = O.subs(sin(theta1+theta5+theta6), Symbol('s156'))
    O = O.subs(sin(theta2+theta3+theta4), Symbol('s234'))
    O = O.subs(sin(theta2+theta3+theta5), Symbol('s235'))
    O = O.subs(sin(theta2+theta3+theta6), Symbol('s236'))
    O = O.subs(sin(theta3+theta4+theta5), Symbol('s345'))
    O = O.subs(sin(theta3+theta4+theta6), Symbol('s346'))
    O = O.subs(sin(theta4+theta5+theta6), Symbol('s456'))
    O = O.subs(sin(theta1-theta2), Symbol('s_{(1-2)}'))
    O = O.subs(sin(theta1-theta3), Symbol('s_{(1-3)}'))
    O = O.subs(sin(theta1-theta4), Symbol('s_{(1-4)}'))
    O = O.subs(sin(theta1-theta5), Symbol('s_{(1-5)}'))
    O = O.subs(sin(theta1-theta6), Symbol('s_{(1-6)}'))
    O = O.subs(sin(theta2-theta3), Symbol('s_{(2-3)}'))
    O = O.subs(sin(theta2-theta4), Symbol('s_{(2-4)}'))
    O = O.subs(sin(theta2-theta5), Symbol('s_{(2-5)}'))
    O = O.subs(sin(theta2-theta6), Symbol('s_{(2-6)}'))
    O = O.subs(sin(theta3-theta4), Symbol('s_{(3-4)}'))
    O = O.subs(sin(theta3-theta5), Symbol('s_{(3-5)}'))
    O = O.subs(sin(theta3-theta6), Symbol('s_{(3-6)}'))
    O = O.subs(sin(theta4-theta5), Symbol('s_{(4-5)}'))
    O = O.subs(sin(theta4-theta6), Symbol('s_{(4-6)}'))
    O = O.subs(sin(theta5-theta6), Symbol('s_{(5-6)}'))
    O = O.subs(sin(theta1+theta2), Symbol('s12'))
    O = O.subs(sin(theta1+theta3), Symbol('s13'))
    O = O.subs(sin(theta1+theta4), Symbol('s14'))
    O = O.subs(sin(theta1+theta5), Symbol('s15'))
    O = O.subs(sin(theta1+theta6), Symbol('s16'))
    O = O.subs(sin(theta2+theta3), Symbol('s23'))
    O = O.subs(sin(theta2+theta4), Symbol('s24'))
    O = O.subs(sin(theta2+theta5), Symbol('s25'))
    O = O.subs(sin(theta2+theta6), Symbol('s26'))
    O = O.subs(sin(theta3+theta4), Symbol('s34'))
    O = O.subs(sin(theta3+theta5), Symbol('s35'))
    O = O.subs(sin(theta3+theta6), Symbol('s36'))
    O = O.subs(sin(theta4+theta5), Symbol('s45'))
    O = O.subs(sin(theta4+theta6), Symbol('s46'))
    O = O.subs(sin(theta5+theta6), Symbol('s56'))
    O = O.subs(sin(theta1), Symbol('s1'))
    O = O.subs(sin(theta2), Symbol('s2'))
    O = O.subs(sin(theta3), Symbol('s3'))
    O = O.subs(sin(theta4), Symbol('s4'))
    O = O.subs(sin(theta5), Symbol('s5'))
    O = O.subs(sin(theta6), Symbol('s6'))

    return O

class Robot():
    T = []
    rotational = []
    H = []
    def __init__(self, DH=None):
        if(DH):
            for i in DH:
                self.T.append(A(i[0], i[1], i[2], i[3]))
                rot = False
                pris = False
                try: 
                    float(i[0])
                except:
                    rot = True
                try:
                    float(i[1])
                except:
                    pris = True
                if(rot or pris):
                    self.rotational.append(rot)
    
    def addDHLine(self, ti, di, ai, ali):
        self.T.append(A(ti, di, ai, ali))
        rot = False
        pris = False
        try: 
            float(ti)
        except:
            rot = True
        try:
            float(di)
        except:
            pris = True
        if(rot or pris):
            self.rotational.append(rot)

    def addNonDHLine(self, dx, dy, dz, Rx, Ry, Rz):
        O = Identity(4)
        O = O@Matrix([[1, 0, 0, dx], [0, 1, 0, dy], [0, 0, 1, dz], [0, 0, 0, 1]]) #Translação
        O = O@Matrix([[1, 0, 0, 0], [0, cos(Rx), -sin(Rx), 0], [0, sin(Rx), cos(Rx), 0], [0, 0, 0, 1]]) #Rotação em X
        O = O@Matrix([[cos(Ry), 0, sin(Ry), 0], [0, 1, 0, 0], [-sin(Ry), 0, cos(Ry), 0], [0, 0, 0, 1]]) #Rotação em Y
        O = O@Matrix([[cos(Rz), -sin(Rz), 0, 0], [sin(Rz), cos(Rz), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) #Rotação em Z
        O.simplify()
        self.T.append(O)
        rot = False
        pris = False
        try: 
            float(Rz)
        except:
            rot = True
        try:
            float(dz)
        except:
            pris = True
        if(rot or pris):
            self.rotational.append(rot)
    
    def HTM(self, a, b, short=True):
        if((a==0) and (b==len(self.T))):
            if(len(self.H)):
                return abrev(self.H)
        O = Identity(4)
        for i in range(b-a):
            O = simplify(O@self.T[a+i])
        if((a==0) and (b==len(self.T))):
            self.H = arredNUM(O)
        if(short):
            return abrev(O)
        else:
            return O

    def POSE(self, joints, links=None):
        if(not len(self.H)):
            pose = Identity(4)
            for i in range(len(joints)):
                pose = simplify(pose@self.T[i])
            self.H = pose
        else:
            pose = self.H
        if(links):
            for i in range(len(links)):
                pose = pose.subs(ls[i], links[i])
        for i in range(len(joints)):
            if(self.rotational[i]):
                pose = pose.subs(thetas[i], joints[i])
            else:
                pose = pose.subs(ds[i], joints[i])
        pose = pose.evalf().simplify()
        return arredNUM(pose)