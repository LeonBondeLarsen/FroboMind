#!/usr/bin/env python 
#-*- coding: utf-8 -*-
'''
Created on Tue Dec  2 10:44:16 2014

@author: frederik
'''

import cv2
from gabor_linear import realKernel
import numpy as np

class line_and_cross_detector:
    
    def __init__ (self,kernel, sigma):

        kernel_size = 60*2+1
        kernel_sigma = 36*6+1
        kernel_psi = 180
        kernel_gamma = 10
        
        kernel_lambda = 5
        kernel_size = int(  50*2  )+1
        kernel_sigma = int( 36*6  )+1

        self.kernel_list = []
        theta_list = []
        
        for r in range(12):
            theta_list.append( r*180/12 )
#        print theta_list
        for theta in theta_list:        
            self.kernel_list.append( realKernel(kernel_size, kernel_sigma, theta, kernel_lambda, kernel_psi, kernel_gamma) )
            

    def analyze_image(self, image):
        try:        

            image = cv2.pyrDown( cv2.pyrDown( image ) )
            gabor_matches = []
            best_image = None
            best_match = 0
            for gabor_kernel in self.kernel_list:
                
                
                gabor_image1 = gabor_kernel.analyze(image)
                gabor_matches.append(gabor_image1)
    
            best_image = None
            best_match = 0
            
            for n, gimage in enumerate(gabor_matches):
                if np.max(gimage) > best_match:
                    best_image = gimage
                    best_match = np.max(gimage)
                    pos = n
    #                    print pos*15                 
      
            ret,thresh = cv2.threshold(np.uint8(best_image),50,255,0)
    #        cv2.imshow("best match t", thresh)
    #                    first_thresh = thresh.copy()
#            ret_image, contours1, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)   
            contours1, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)                     
    #        print len(contours1)
    #                    for x in contours:
    #                    print contours
            areas = [cv2.contourArea(c) for c in contours1]
            max_index1 = np.argmax(areas)
            cnt1=contours1[max_index1]
    
            rect = cv2.minAreaRect(cnt1)
    #        print "rect", rect
            if rect[1][0]*rect[1][1] < 300:
                return None
#            box = cv2.boxPoints(rect)
#            box = np.int0(box)
#            cv2.drawContours(image,[box],0,(0,255,0),2)
            
            
                   
            if pos > 5:
                second_best = gabor_matches[pos - 6]
                second_best_match = np.max(gabor_matches[pos - 6])
            else:
                second_best_match = np.max(gabor_matches[pos + 6])
                second_best = gabor_matches[pos + 6]
            if second_best_match > best_match * 0.7 :  
    #            print "Second best" 
                ret, thresh2 = cv2.threshold(np.uint8(second_best),50,255,0)
    #                        second_thresh = thresh2.copy()
    #            cv2.imshow("second best match t", thresh2)
#                ret_image, contours2, hierarchy = cv2.findContours(thresh2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) 
                contours2, hierarchy = cv2.findContours(thresh2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)                   
                areas = [cv2.contourArea(c) for c in contours2]
                max_index2 = np.argmax(areas)
                cnt2=contours2[max_index2]
                rect2 = cv2.minAreaRect(cnt2)
    #            print rect2
                if rect2[1][0]*rect2[1][1] < 300:
    #                print "no second box"
                    return None
#                box = cv2.boxPoints(rect2)
#                box = np.int0(box)
#                cv2.drawContours(image,[box],0,(255,0,0),2)
                """ Here the corner will be found """
    #                        print second_thresh
    #            print image.shape
                first_thresh = np.zeros((120,160,1))
                second_thresh = np.zeros((120,160,1))
                
                cv2.drawContours(first_thresh, contours1, max_index1, 1, -1)
                cv2.drawContours(second_thresh, contours2, max_index2, 1, -1)
                #cv2.imshow("corner best match t", first_thresh )                 
#                ret_image, contours, hierarchy = cv2.findContours( np.array( first_thresh * second_thresh, dtype=np.uint8),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)                   
                contours, hierarchy = cv2.findContours( np.array( first_thresh * second_thresh, dtype=np.uint8),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)                   

                areas = [cv2.contourArea(c) for c in contours]
                max_index = np.argmax(areas)

                cnt=contours[max_index]
                rect3 = cv2.minAreaRect(cnt)
    #            print "corner found", rect3
    #            return rect3[0] , rect[2], rect2[2]
                cv2.circle(image, ( int(rect3[0][0]), int(rect3[0][1])), 5, (255,255,0), -1)
            else:
                return None
    #            print "no second box"
                                
                #cv2.imshow("second best match", second_best)    
            #cv2.imshow('input_image',image)                
                
            #cv2.imshow("best match",cv2.pyrUp( cv2.pyrUp(image)))
    
            if rect[1][0] < rect[1][1]:
                rotation1 = rect[2] + 180
            else:
                rotation1 = rect[2] + 90
    
            if rect2[1][0] < rect2[1][1]:
                rotation2 = rect2[2] + 180
            else:
                rotation2 = rect2[2] + 90
    
            
    
            return rect3[0] , rotation1, rotation2
        except:
            #print "something failed"
            return None



