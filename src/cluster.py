import numpy as np
from sklearn.cluster import DBSCAN
import pandas as pd
import tools

def clusterize_parallel(linesArray,eps,min_sample):
	max_clusters = 10
	normal_vector = linesArray[:,5:7]
	try:
		db = DBSCAN(eps, min_sample).fit(normal_vector)
		labels = db.labels_
		n_clusters_ = max(set(labels))
		clusters = pd.Series([linesArray[labels == i] for i in xrange(0,n_clusters_+1)]) 
		tab_ab=[]	
		for cluster in clusters:
			a = np.mean(cluster[:,5]) 
			b = np.mean(cluster[:,6]) 
			tab_ab.append([a,b])
		return (np.array(tab_ab),clusters, labels)
	except Exception as e:
		print("Error")

def clusterize_xy(linesArray,eps,min_sample):
	#first, we compute the array of midlles of each segment 
	midXY = tools.findMiddlePointsArray(linesArray)
	# we will clusterize on y(x) so we write y first
	mid=np.concatenate((np.expand_dims(midXY[:,1],axis=1),np.expand_dims(midXY[:,0],axis=1)), axis =1)
	try:
		db = DBSCAN(eps, min_sample).fit(mid)
		labels = db.labels_
		n_clusters_ = max(set(labels))
		clusters = pd.Series([linesArray[labels == i] for i in xrange(0,n_clusters_+1)]) 
		return(clusters,labels)
	except Exception as e:
		print("Error")

def clusterize_offsets(linesArray,eps,min_samples):
	vecteur_c = linesArray[:,7]
	n_lines = np.size(vecteur_c)
	X = vecteur_c.reshape(n_lines,1)
	db = DBSCAN(eps, min_samples).fit(X)##
	labels = db.labels_ #-1
	nb_cluster = max(labels)
	i_list = list(np.arange(nb_cluster+1))
	gathered_on_C = pd.Series([linesArray[labels == i] for i in i_list])
	tab_c=[]
	for cluster in gathered_on_C:
		c = np.mean(cluster[:,7]) 
		tab_c.append(c)
	# we add the well detected lines (label == -1) at the end:
	if -1 in set(labels):
		well_detected_lines = linesArray[labels == -1]
		new_indexes = list(np.arange(nb_cluster+1 , nb_cluster+len(well_detected_lines)+1))
		k = nb_cluster + 1
		for line in well_detected_lines:
			tab_c.append(line[7])
			i_list.append(k)
			k = k+1
		s1 = pd.Series([[line] for line in well_detected_lines],index = new_indexes)
		gathered_on_C = gathered_on_C.append(s1)	
	return (tab_c, i_list, gathered_on_C, labels)