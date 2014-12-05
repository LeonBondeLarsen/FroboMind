#!/usr/bin/env python 
#-*- coding: utf-8 -*-
'''
Created on Thu Oct 30 16:36:33 2014

@author: frederik
'''

__author__ = 'Frederik Hagelskjær'
import numpy as np
import cv2


def mkKernel(ks, sig, th , lm, ps, gm):
        """ Check the kernel size"""
        if not ks%2:
            exit(1)

        """ Definition of the varibles"""
        theta = th * np.pi/180.
        psi = ps * np.pi/180.
        sigma = np.float(sig)/ks
        lmbd = np.float(lm)
        gamma = gm/10.0
        
        """Creating the kernel size"""        
        xs=np.linspace(-1*ks/10.,1*ks/10.,ks)
        ys=np.linspace(-1*ks/10.,1*ks/10.,ks)
        
        """Creating the kernel"""        
        x,y = np.meshgrid(xs,ys)        

        """ Angle of the signal """
        x_theta = x*np.cos(theta)+y*np.sin(theta)
        y_theta = -x*np.sin(theta)+y*np.cos(theta)
 
        #return np.array( np.exp(-0.5*(x_theta**2+gamma * y_theta**2)/sigma**2)*np.cos(2.*np.pi*x_theta/lmbd + psi),dtype=np.float32)
                                           
        """  Return the kernel                  The gauss signal                                           The sinus wave                                   """
        return np.array( np.exp(-0.5*(x_theta**2 + (gamma*y_theta)**2)/((sigma)**2)) * np.cos(2.*np.pi*x_theta/lmbd + psi),dtype=np.float32)
	#  - np.exp( sigma / 2)


class realKernel:
    
    def __init__ (self, ks, sig, th, lm, ps, gm):
        self.kernel_real = mkKernel(ks, sig, th, lm, ps, gm )
    
    def analyze(self, image):
        #image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        resizedImage = np.array(image, dtype=np.float32)
        
        redWeight = 2
        greenWeight = 1.3
        blueWeight = 1
        src_f = redWeight * resizedImage[:,:,2] - greenWeight * resizedImage[:,:,1] - blueWeight * resizedImage[:,:,0]
        ret,src_f = cv2.threshold(src_f,100,255,cv2.THRESH_TOZERO)

        
        src_f /= 255.

        dest = cv2.filter2D(src_f, cv2.CV_32F ,self.kernel_real)               
        ret, dest_new = cv2.threshold(-1*dest,50,255,cv2.THRESH_TOZERO)
        #cv2.imshow("qwrw",dest_new)
        #print "dest"
#        print np.max(dest)
#        print np.min(dest)

        return dest_new
        
    def get_kernel(self):
        #print "kernel"
        #print np.max(self.kernel_real)
        #print np.min(self.kernel_real)
        kernelimg = self.kernel_real
        return kernelimg