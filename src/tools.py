from __future__ import division
import numpy as np
from scipy.stats import linregress 
from sklearn import linear_model #donnait la methode des moindre carres et jugee inadaptee
import sklearn.svm
import collections
import pandas as pd
import math
from statistics import median
from operator import truediv

def sign(x):
    if x > 0:
        return 1.
    elif x < 0:
        return -1.
    elif x == 0:
        return 1.
    else:
        return x

def findLength(x1,x2,y1,y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def findParamSegment(segment):
	x1 = segment[0]
	y1 = segment[1]
	x2 = segment[2]
	y2 = segment[3]
	b =-(x2-x1)
	a =(y2-y1)
	c =-(a*x1+b*y1)
	norm = math.sqrt(a**2+b**2)*sign(b) #b sera toujours positif pour eviter toute redondance
	if norm!=0:
		return(np.array([a/norm,b/norm,c/norm]))

def findParamSegments(linesArray):
    if linesArray is None:
        return "no stairs"
    n = np.shape(linesArray)[0]
    x1 = linesArray[:,0,0]
    y1 = linesArray[:,0,1]
    x2 = linesArray[:,0,2]
    y2 = linesArray[:,0,3]
    L = np.sqrt(np.square(x1 -x2) + np.square(y1-y2))
    A = (y2-y1)
    B = -(x2-x1)
    C = -(A*x1+B*y1)
    normalVector = np.concatenate((np.expand_dims(A,axis=1),np.expand_dims(B,axis=1)), axis =1)
    norm = np.linalg.norm(normalVector, axis =1)*np.sign(B) # B positif
    # on supprime DEJA les lignes de norme nulle
    linesArray = linesArray.reshape(n,4)
    linesArray = linesArray[norm != 0]
    normalVector = normalVector[norm != 0]
    L = L[norm !=0]
    C = C[norm !=0]
    norm = norm[norm !=0]
    lines = np.concatenate((linesArray, np.expand_dims(L,axis=1), np.apply_along_axis(np.divide, 0, normalVector,norm), np.expand_dims(np.divide(C,norm),axis=1)),axis=1)
    return lines

def segmentsLongerThan(linesArray, lmin):
    cond = np.greater(linesArray[:,4],lmin)
    longLines = linesArray[cond]
    return longLines

def mergeLines(lines):
    if np.shape(lines)[0]>1:
        a = np.mean(lines[:,5])
        b = np.mean(lines[:,6])
        c = np.mean(lines[:,7])
        if b > 0.1:
            x = np.concatenate((lines[:,0],lines[:,2]),axis= 0)
            x1 = np.min(x)
            x2 = np.max(x)
            y1 = -(a*x1+c)/b # calcul potentiellement dangereux en cas de ligne verticale
            y2 = -(a*x2+c)/b
        else: # alors a est suffisamment grand
            y = np.concatenate((lines[:,1],lines[:,3]),axis= 0)
            y1 = np.min(y)
            y2 = np.max(y)
            x1 = -(b*y1+c)/a
            x2 = -(b*y2+c)/a
        l = findLength(x1,x2,y1,y2)
        extremCoord = [x1,y1,x2,y2,l,a,b,c] 
        return extremCoord
    else:
        return lines[0]

def mergeClusterSerieAfterC(i_list, gathered_on_C,labels_c): # a appliquer a une sortie de lustering
	mergedLinesArray = []
	for i in i_list:
		mergedLinesArray.append(mergeLines(gathered_on_C[i]))
	return np.array(mergedLinesArray) 

def findUpperSide(parallelLinesArray): # a appliquer a un ensemble de lignes paralleles
    ymin1 = np.min(parallelLinesArray[:,1])
    arg_min1 = np.argmin(parallelLinesArray[:,1])
    ymin2 = np.min(parallelLinesArray[:,3])
    arg_min2 = np.argmin(parallelLinesArray[:,3])
    ymin = min(ymin1,ymin2)
    if ymin == ymin1:
        arg_min = arg_min1
    else : 
        arg_min = arg_min2
    return parallelLinesArray[arg_min].astype('float64')

def findLowerSide(parallelLinesArray): # a appliquer a un ensemble de lignes paralleles
    ymax1 = np.max(parallelLinesArray[:,1])
    arg_max1 = np.argmax(parallelLinesArray[:,1])
    ymax2 = np.max(parallelLinesArray[:,3])
    arg_max2 = np.argmax(parallelLinesArray[:,3])
    ymax = max(ymax1,ymax2)
    if ymax == ymax1:
        arg_max = arg_max1
    else : 
        arg_max = arg_max2
    return parallelLinesArray[arg_max].astype('float64')

def findLinesIntersectionPoint(line1,line2):
    a1=line1[0]
    b1=line1[1]
    c1=line1[2]
    a2=line2[0]
    b2=line2[1]
    c2=line2[2]
    y = (a1*c2 - a2*c1)/(b1*a2-b2*a1)
    x = ((b1-b2)*y + c1-c2)/(a2-a1)
    return(x,y)

def findLineIntersectionPoints(line,linesArray):
    n = np.shape(linesArray)[0]
    a=line[0]
    b=line[1]
    c=line[2]
    A=linesArray[5]
    B=linesArray[6]
    C=linesArray[7]
    y = (a*C - A*c)/(b*A-B*a)
    x = ((b*np.ones((n,1))-B)*y + c*np.ones((n,1))-C)/(A-a*np.ones((n,1)))
    return np.concatenate((x,y),axis=1)

def linearRegressionCoeff(pointsArray):
    regr = linear_model.LinearRegression()
    Xarray = pointsArray[:,0]
    Yarray = pointsArray[:,1]
    l = np.size(Xarray)
    if (abs(max(Xarray)-min(Xarray))) > (abs(max(Yarray)-min(Yarray))):
        #si la droite est globalement horizontale, LinearRegression ne bug pas
        regr.fit(Xarray.reshape((l, 1)),Yarray)
        m = regr.coef_[0]#regr.coef_ est un tableau a 1 dim avec linear reg et a 2 dim avec svm
        p = regr.intercept_
        #y=mx+p
        div = math.sqrt(1+m**2)
        a = -m/div
        b = 1/div#b toujours positif
        c = -p/div
    else: #si la droite est globalement verticale, LinearRegression bug
        regr.fit(Yarray.reshape((l, 1)),Xarray)
        m = regr.coef_[0]
        p = regr.intercept_
        #x=my+p
        div = math.sqrt(1+m**2)*sign(m)#b doit toujours etre positif
        a = -1/div
        b = m/div#b toujours positif
        c = p/div
    print(m,p)
    return [a,b,c]

def linearRegressionSVRCoeff(pointsArray):
    regr = sklearn.svm.SVR(kernel='linear', C=100, epsilon=30)
    Xarray = pointsArray[:,0]
    Yarray = pointsArray[:,1]
    l = np.size(Xarray)
    #if (abs(max(Xarray)-min(Xarray))) > (abs(max(Yarray)-min(Yarray)))*1.2:
    #    #si la droite est globalement horizontale, LinearRegression ne bug pas
    #    #facteur 1.2 empirique, rajoute pour le cas ou l'escalier prend toute la largeur de l'image
    #    regr.fit(Xarray.reshape((l, 1)),Yarray)
    #    m = regr.coef_[0][0]#regr.coef_ est un tableau a 1 dim avec linear reg et a 2 dim avec svm
    #    p = regr.intercept_[0]
    #    #y=mx+p
    #    div = math.sqrt(1+m**2)
    #    a = -m/div
    #    b = 1/div#b toujours positif
    #    c = -p/div
    #else: #si la droite est globalement verticale, LinearRegression bug
    regr.fit(Yarray.reshape((l, 1)),Xarray)
    m = regr.coef_[0][0]
    p = regr.intercept_[0]
    #x=my+p
    div = math.sqrt(1+m**2)*sign(m)#b doit toujours etre positif
    a = -1/div
    b = m/div#b toujours positif
    c = p/div
    print(m,p)
    return [a,b,c]

def findLeftRightPoints(linesArray):
    tableauRangPointDroit = np.greater(linesArray[:,2],linesArray[:,0])#1 si x2 a droite
    rightXarray = np.where(tableauRangPointDroit,linesArray[:,2],linesArray[:,0])
    rightYarray = np.where(tableauRangPointDroit,linesArray[:,3],linesArray[:,1])
    leftXarray = np.where(tableauRangPointDroit,linesArray[:,0],linesArray[:,2])
    leftYarray = np.where(tableauRangPointDroit,linesArray[:,1],linesArray[:,3])
    rightPointsArray = np.concatenate((np.expand_dims(rightXarray,axis=1),np.expand_dims(rightYarray,axis=1)),axis=1)
    leftPointsArray = np.concatenate((np.expand_dims(leftXarray,axis=1),np.expand_dims(leftYarray,axis=1)),axis=1)
    return leftPointsArray, rightPointsArray
    #return leftPointsArray.astype('float32'), rightPointsArray.astype('float32')

def findTrapeze(mergedLinesArray,chosen_regression_string = 'lin'): #'lin' ou 'svr'
    n = np.shape(mergedLinesArray)[0]
    leftPointsArray, rightPointsArray = findLeftRightPoints(mergedLinesArray)
    midPointsArray = findMiddlePointsArray(mergedLinesArray)
    midLineCoef = linearRegressionCoeff(midPointsArray)

    goodlinesLeftPoints = leftPointsArray
    goodlinesRightPoints = rightPointsArray
    # ELIMINE LES POINTS DONT LES MILIEUX SONT TROP LOINS DE LA VALEUR THEORIQUE
    #estimated_mid = findLineIntersectionPoints(midLineCoef, mergedLinesArray)
    #mid_deviation = midPointsArray - estimated_mid
    #mid_mean = np.mean(mid_deviation)#should be 0
    #mid_variance  = np.var((mid_deviation), axis = 1)
    #print("mid mean, var")
    #print(mid_mean,mid_variance)
    #cond_gch = np.greater(mid_deviation)
    
    #  ELIMINE LES POINTS QUI SE SITUENT A UNE DISTANCE TROP GRANDE DE LA MOYENNE
    #goodlinesLeftPoints = np.empty((0, 2), dtype=np.float32)
    #goodlinesRightPoints = np.empty((0, 2), dtype=np.float32)
    #for i in range(n):
    #    center_strs = findLinesIntersectionPoint(midLineCoef,mergedLinesArray[i,5:])[0]#x coordinate of the theoretical center of the stair i
    #    print("i,left,mid,right")
    #    print(i,leftPointsArray[i,0],center_strs,rightPointsArray[i,0])
    #    if leftPointsArray[i,0] < center_strs*0.5 and rightPointsArray[i,0] > center_strs*1.5:
    #        goodlinesLeftPoints = np.concatenate((goodlinesLeftPoints,np.expand_dims(leftPointsArray[i],axis=0)),axis=0)
    #        goodlinesRightPoints = np.concatenate((goodlinesRightPoints,np.expand_dims(rightPointsArray[i],axis=0)),axis=0)
    
    middleLine = np.expand_dims(np.concatenate((findLinesIntersectionPoint(midLineCoef, mergedLinesArray[0,5:]),findLinesIntersectionPoint(midLineCoef, mergedLinesArray[n-1,5:]))),axis=0)
    if chosen_regression_string == 'lin':
        leftSide = linearRegressionCoeff(goodlinesLeftPoints)
        rightSide = linearRegressionCoeff(goodlinesRightPoints)
    elif chosen_regression_string == 'svr':
        leftSide = linearRegressionSVRCoeff(goodlinesLeftPoints)
        rightSide = linearRegressionSVRCoeff(goodlinesRightPoints)
    upperSide = findUpperSide(mergedLinesArray)
    lowerSide = findLowerSide(mergedLinesArray)
    upperSide = upperSide[5:]
    lowerSide = lowerSide[5:]
    vertex = []
    vertex.append(findLinesIntersectionPoint(rightSide, upperSide))#en haut a droite
    vertex.append(findLinesIntersectionPoint(leftSide, upperSide))#en haut a gauche
    vertex.append(findLinesIntersectionPoint(leftSide, lowerSide))#en bas a gauche
    vertex.append(findLinesIntersectionPoint(lowerSide, rightSide))#en bas a droite
    rightline = [vertex[0][0],vertex[0][1],vertex[3][0],vertex[3][1]]
    leftline  = [vertex[1][0],vertex[1][1],vertex[2][0],vertex[2][1]]
    
    if chosen_regression_string=='svr':
        epsilonSVR = 30
        upperline = [vertex[0][0]-epsilonSVR,vertex[0][1],vertex[1][0]+epsilonSVR,vertex[1][1]]
        lowerline = [vertex[2][0]-epsilonSVR,vertex[2][1],vertex[3][0]+epsilonSVR,vertex[3][1]]
        r1 = [vertex[0][0]-epsilonSVR,vertex[0][1],vertex[3][0]-epsilonSVR,vertex[3][1]]
        r2 = [vertex[0][0]+epsilonSVR,vertex[0][1],vertex[3][0]+epsilonSVR,vertex[3][1]]
        l1 = [vertex[1][0]-epsilonSVR,vertex[1][1],vertex[2][0]-epsilonSVR,vertex[2][1]]
        l2 = [vertex[1][0]+epsilonSVR,vertex[1][1],vertex[2][0]+epsilonSVR,vertex[2][1]]
        trapezeLinesArray = findParamSegments(np.reshape([rightline,upperline,leftline,lowerline, r1, r2, l1, l2 ],(8,1,4)))
    else:
        upperline = [vertex[0][0],vertex[0][1],vertex[1][0],vertex[1][1]]
        lowerline = [vertex[2][0],vertex[2][1],vertex[3][0],vertex[3][1]]
        trapezeLinesArray = findParamSegments(np.reshape([rightline,upperline,leftline,lowerline],(4,1,4)))
    return trapezeLinesArray, goodlinesLeftPoints, goodlinesRightPoints, middleLine # seul trapezeLinesArray est vraiment utile, le reste pourra etre supprime dans une implementation reelle

def findLabelStairs(tab_ab, clustersAB, labels, angle_min, angle_max):
    n = np.shape(clustersAB)[0]
    labelsList=[]
    sizesList=[]
    for i in range (n):##?
        if np.arctan2(tab_ab[i][1],tab_ab[i][0]) > angle_min and np.arctan2(tab_ab[i][1],tab_ab[i][0]) < angle_max:
            labelsList.append(i)
            sizesList.append(len(clustersAB[i]))
    if  labelsList == []: # si pas d'escalier
        return "NoStairs" # risque de confusion avec label = 0 si on met false
    i_max =  sizesList.index(max( sizesList))
    return labelsList[i_max] 

def findMiddle(linesArray): #a appliquer au cluster stairs pour trouver son centre
    x = [linesArray[:,0],linesArray[:,2]]
    y = [linesArray[:,1],linesArray[:,3]]
    meanX = np.mean(x)
    meanY = np.mean(y)
    return meanX, meanY

def findMiddlePointsArray(linesArray): # a appliquer au cluster stairs pour trouver les centres de chaque marche
    x = np.concatenate((np.expand_dims(linesArray[:,0],axis=1),np.expand_dims(linesArray[:,2],axis=1)),axis=1)
    y = np.concatenate((np.expand_dims(linesArray[:,1],axis=1),np.expand_dims(linesArray[:,3],axis=1)),axis=1)
    meansX = np.mean(x,axis=1)
    meansY = np.mean(y,axis=1)
    return np.concatenate((np.expand_dims(meansX,axis=1),np.expand_dims(meansY,axis=1)),axis=1)

def findLowestStair(linesArray):
    y = [linesArray[:,1],linesArray[:,3]]
    return np.max(y)

def findLowerStairs(stairsLinesArray,lowest_st_nb):
    length = len(stairsLinesArray)
    if length > lowest_st_nb:
        lowest_stairs_lines = stairs_lines[length-lowest_st_nb:length,:]
    else:
        lowest_stairs_lines = stairs_lines
    return lowest_stairs_lines

# unused exceptions
class NoStairs(Exception):
    def __init__(self):
        Exception.__init__(self,"no stairs detected")

class NotEnoughlines(Exception):
    def __init__(self):
        Exception.__init__(self,"not enough lines detected, impossible to compute operations on these lines")