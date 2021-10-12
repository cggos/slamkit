#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Distribution():
    def __init__(self,mu,Sigma):
        self.mu = mu
        self.sigma = Sigma

    def tow_d_gaussian(self,x):
        mu = self.mu
        Sigma =self.sigma
        n = mu.shape[0]
        Sigma_det = np.linalg.det(Sigma)
        Sigma_inv = np.linalg.inv(Sigma)
        N = np.sqrt((2*np.pi)**n*Sigma_det)

        fac = np.einsum('...k,kl,...l->...',x-mu,Sigma_inv,x-mu)

        return np.exp(-fac/2)/N

    def one_d_gaussian(self,x):
        mu = self.mu
        sigma = self.sigma

        N = np.sqrt(2*np.pi*np.power(sigma,2))
        fac = np.power(x-mu,2)/np.power(sigma,2)
        return np.exp(-fac/2)/N


if __name__=='__main__':

    p1 = Distribution(0,2)
    x = np.linspace(-10,10,100)
    y = p1.one_d_gaussian(x)
    plt.plot(x,y,'b-',linewidth=3)
    # plt.show()

    N = 60
    X = np.linspace(-3,3,N)
    Y = np.linspace(-4,4,N)
    X,Y = np.meshgrid(X,Y)
    mu = np.array([0.,0.])
    Sigma = np.array([[1.,-0.5],[-0.5,1.5]])
    pos = np.empty(X.shape+(2,))
    pos[:,:,0]= X
    pos[:,:,1] = Y

    p2 = Distribution(mu,Sigma)
    Z = p2.tow_d_gaussian(pos)

    fig =plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot_surface(X,Y,Z,rstride=3,cstride=3,linewidth=1,antialiased =True)
    cset = ax.contour(X,Y,Z,zdir='z',offset=-0.15)

    ax.set_zlim(-0.15,0.2)
    ax.set_zticks(np.linspace(0,0.2,5))
    ax.view_init(27,-21)
    plt.show()

