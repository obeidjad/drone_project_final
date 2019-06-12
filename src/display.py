import cv2
import matplotlib.pyplot as plt
import numpy as np
import cluster

############################# OUTILS DE DESSIN ################################
def segments(img, nameString, lines, color,thickness,lsd):
  drawn_img=lsd.drawSegments(img,lines)

def draw_line (screen, nameString, line, color, thickness):
  H, W = screen.shape[:2]
  a=line[5]
  b=line[6]
  c=line[7]
  T=[]

  def f(x,y):
    return(a*x+b*y+c)

  if f(0,0)*f(W,0)<0: #croise le bord haut de la fenetre
    x=(-c/a)
    y=0
    T.append(x,y)
  if f(W,0)*f(W,H)<0: #croise le bord droit de la fenetre
    x=W
    y=(-c-a*W)/b
    T.append(x,y)
  if f(0,H)*f(W,H)<0: #croise le bord en bas 
    x=(-c-b*H)/a
    y=H
    T.append(x,y)
  if f(0,0)*f(0,H)<0:
    x=0
    y=-c/b
    T.append(x,y)
  cv2.line(screen, (int(T[0]),int(T[1])),(int(T[2]),int(T[3])), color, thickness, lineType=8, shift=0)

def draw_lines(drawn_img, nameString, lines, color, thickness):
  for line in lines:
    drawn_img = draw_line(drawn_img, nameString, line,color, thickness)

def draw_segments_couleur_angle(drawn_img, lines, tab_ab, labels, clusters_AB,thickness):
  i=0
  for line in lines:
    label_i=labels[i]#+1
    color=cluster_color(tab_ab,label_i)
    drawn_img = cv2.line(drawn_img, (int(line[0]),int(line[1])),(int(line[2]),int(line[3])), color, thickness, lineType=8, shift=0)
    i+=1

def draw_segments_couleur_angle_par_cluster(drawn_img, lines, tab_ab, labels, clusters_AB,thickness):
  i=0
  for label in labels:#on parcourt les labels
    color=cluster_color(tab_ab,label)
    lines_to_plot=clusters_AB[label]
    for line in lines_to_plot:#on parcourt le cluster 
      drawn_img = cv2.line(drawn_img, (int(line[0]),int(line[1])),(int(line[2]),int(line[3])), color, thickness, lineType=8, shift=0)

def draw_segments(drawn_img, lines, color,thickness):
  i=0
  for line in lines:
    drawn_img = cv2.line(drawn_img, (int(line[0]),int(line[1])),(int(line[2]),int(line[3])), color, thickness, lineType=8, shift=0)
    i+=1

def draw_segment(drawn_img, line, color_tab, thickness):
  drawn_img = cv2.line(drawn_img, (int(line[0]),int(line[1])),(int(line[2]),int(line[3])), color_tab, thickness, lineType=8, shift=0)

def points(img, pointsArray, radius = 3, color = 255,thickness=-1):#j'ai supprime l'argument nameString en deuxieme position
  drawn_img = img
  for x,y in pointsArray:
    drawn_img = cv2.circle(img, (int(x),int(y)), radius, color, thickness)
    #cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 

####################### CLUSTERS #########################################################################
def clusters(vecteur_normalise,labels,desc_string,a=None, b=None):
  #utilise plot, cas general
	fig, ax = plt.subplots()
	ax.scatter(vecteur_normalise[:,0], vecteur_normalise[:,1], c=labels)
	if desc_string == "ab":
	  ax.set_xlabel('a norm')
	  ax.set_ylabel('b norm')
	  plt.title('repartion b en fonction de a')
	elif desc_string == "c":
	  ax.set_xlabel('c')
	  ax.set_ylabel('c')
	  title = 'repartion de c sur le cluster ab, a=' + str(a) + ' b=' +str(b)
	  plt.title(title)
	else:
		print("vous devez quel type de cluster il faut afficher, rentrez ab ou c - en precisant a et b dans ce cas")
	plt.show()

def empty_cluster(desc_string="ab", a=None, b=None):
  #preciser le type de cluster: "ab"(par defaut) ou "c"
  #cree le fond
  if desc_string == "ab":
    name = "repartion b en fonction de a"
    x_name = "a norm"
    y_name = "b norm"
  if desc_string == "c":
    name = "repartion de c sur le cluster ab, a=" + str(a) + " b=" +str(b)
    x_name = "c"
    y_name = "c"
  font                   = cv2.FONT_HERSHEY_SIMPLEX
  fontScale              = 1
  fontColor              = 0
  lineType               = 2#cv2.LINE_AA
  height, width          =  200, 200
  mid = height//2
  if desc_string=="ab":
    x_mid = mid
    y_mid = mid
  if desc_string=="c":
    x_mid = mid
    y_mid = mid
  empty_window(name)
  img_plot = empty_white_img (height,width)
  #on commence par nommer l'axe y et faire pivoter l'image
  cv2.putText(img_plot, y_name, (70,x_mid-20), font, fontScale, fontColor, lineType)
  rows,cols,color_type = img_plot.shape
  M = cv2.getRotationMatrix2D((cols/2,rows/2),90,1)
  img_plot = cv2.warpAffine(img_plot,M,(cols,rows))
  # on place les autres elements
  mid = height//2
  cv2.arrowedLine(img_plot, (x_mid, y_mid), (680, y_mid), 0, thickness=1, line_type=8,  shift=0, tipLength=0.1)
  cv2.arrowedLine(img_plot, (x_mid, y_mid), (x_mid,680), 0, thickness=1, line_type=8,  shift=0, tipLength=0.1)
  cv2.putText(img_plot, x_name, (width-200, y_mid+30), font, fontScale, fontColor, lineType) #img,text,coord,font
  return name, img_plot

def cluster_plot(vecteur_normalise,labels, image_plot, name, desc_string="ab"):
  #remplit le fond avec les donnees
  height, width =  200, 200
  mid = height//2
  if desc_string=="ab":
    x_mid = mid-50
    y_mid = mid
    scale = 80
    cluster_plot = image_plot.copy()
  if desc_string=="c":
    x_mid = mid
    y_mid = mid
    scale = 800
    cluster_plot = image_plot
  n = vecteur_normalise.shape[0]
  radius = 3
  thickness=-1
  for i in range (n):
    cluster_plot = cv2.circle(cluster_plot,(int(vecteur_normalise[i,0]*scale)+x_mid,int(vecteur_normalise[i,1]*scale)+y_mid), radius, colors(labels[i]+1),thickness)
  cv2.imshow(name,cluster_plot)# ou return cluster_plot

def cluster_plot_bis(vecteur_normalise,cluster_AB, tab_ab, labels, image_plot, name, desc_string="ab"):
  #remplit le fond avec les donnees
  height, width =  200, 200
  mid = height//2
  if desc_string=="ab":
    x_mid = mid-50
    y_mid = mid
    scale = 80
    cluster_plot = image_plot.copy()
  if desc_string=="c":
    x_mid = mid
    y_mid = mid
    scale = 80
    cluster_plot = image_plot
  n = vecteur_normalise.shape[0]
  radius = 3
  thickness=-1
  n=np.max(labels)
  i=0

  for label_i in labels:
    if label_i>-1:
      cluster_AB_i=cluster_AB[label_i]
      color=cluster_color(tab_ab,label_i)
      vecteur_plot=cluster_AB_i[:,5:7]
      for vecteur_normalises in vecteur_plot:
        cluster_plot = cv2.circle(cluster_plot,(int(vecteur_normalises[0]*scale)+2*x_mid,int(vecteur_normalises[1]*scale)+y_mid), radius, color,thickness)
  image_plot=cluster_plot
  cv2.imshow(name,cluster_plot)# ou return cluster_plot*
  cv2.waitKey(10)

def cluster_ab(vecteur_normalise,labels, image_plot, window_name):
  cluster_plot(vecteur_normalise,labels, image_plot, window_name, "ab")

def cluster_ab_bis(vecteur_normalise,cluster_AB, tab_ab,labels, image_plot, window_name):
  cluster_plot_bis(vecteur_normalise,cluster_AB, tab_ab, labels, image_plot, window_name, "ab")

def cluster_c(vecteur_normalise,labels, a, b):
  vecteur_normalise = np.apply_along_axis(np.divide,0,vecteur_normalise,np.max(np.abs(vecteur_normalise)))#est ce que ca ne modifierait pas le tableau initial?
  name, img_plot = empty_cluster("c", a, b)
  cluster_plot(vecteur_normalise,labels, img_plot, name, "c")

def couleur():
  colors=np.zeros((180,3))
  for i in range (60):
    colors[i][2]=255-i*4
    colors[i+30][1]=4*i
    colors[179-i-30][1]=4*i
    colors[179-i][0]=255-4*i
  return(colors)

def colors_angle(angle):
  colors = couleur()
  color=tuple((colors[angle,0],colors[angle,1],colors[angle,2]))
  return(color)

def cluster_color(tab_ab,i):
  a_moy=tab_ab[i,0]
  b_moy=tab_ab[i,1]
  if abs(a_moy)>0.01:
		teta=int((np.arctan(b_moy/a_moy)*180)/(np.pi))
		color=colors_angle(teta)
  else:
		color=colors_angle(1)
  return(color)


####################### GESTION DES FENETRES #####################
def empty_white_img(height,width):
  blank_image = np.ones((height,width,3), np.float32)*255#initialement np.uint8 #cv2.waitKey(0)np.float32?
  return blank_image

def empty_black_img(height,width):
  blank_image = np.zeros((height,width,3), np.float32)#initialement np.uint8 #cv2.waitKey(0)np.float32?
  return blank_image

def copy_img(img):
  drawn_img = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)#a sortir pour pouvoir afficher plusieur
  return drawn_img

def empty_window(nameString):
  cv2.namedWindow(nameString, cv2.WINDOW_KEEPRATIO)

def resize_window(nameString, img):
  screen_res = 1280.0, 720.0
  scale_width = screen_res[0] / img.shape[1]
  scale_height = screen_res[1] / img.shape[0]
  scale = min(scale_width, scale_height)
  window_width = int(img.shape[1] * scale)
  window_height = int(img.shape[0] * scale)
  cv2.namedWindow(nameString, cv2.WINDOW_NORMAL) 
  cv2.resizeWindow(nameString, window_width, window_height)
  return window_width, window_height

#################### GESTION colors GENERALE #######################
def colors(index):
  c = [(0, 0, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255), (255, 0, 255), (120, 120, 120), (120, 50, 0), (50, 120, 0), (0, 50, 120), (120, 0, 50), (0,120,50),(50, 0, 120)]
  if index >= len(c):
    return c[0] 
  else:
   return c[index]

def colors_table(labels):
  return [colors(label + 1) for label in labels]

def different_window(img, nameString, function, argArray, color, thickness):
  drawn_img = copy_img(img)
  function(drawn_img, argArray, color, thickness)
  resize_window(nameString,img)
  cv2.imshow(nameString,drawn_img)