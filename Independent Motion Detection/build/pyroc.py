from matplotlib.pyplot import figure, show
from matplotlib import pyplot as plt
import numpy as npy
from numpy.random import rand
import math


# read csv data
import csv

ball_array = []
distance_array = []
csvfiles = ['positive_example.csv']#,'positive_example_2.csv'];
for file1 in csvfiles:
    with open(file1, 'rb') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',')
        for row in spamreader:
            #print row[0], row[1]
            ball_array.append( row[0])
            distance_array.append(float ( row[1]))

popsize = len(ball_array)
print "total points = " ,popsize

#print ball_array
#print distance_array

true_rate = []
false_rate = []

# now iterate for everythreshold
tmax= 60
threshold_array = range(1 , tmax)
for threshold in threshold_array:
    
  
    ball_count = 0
    ball_outliers = 0
    background_outliers = 0
    for j in range(1, popsize):
        #print distance_array[j]
        #determine the number of ball corners
        if ball_array[j] == '1':
            ball_count = ball_count + 1
            #determine how many ball corners are thresholds 
            if distance_array[j] > threshold:
                ball_outliers = ball_outliers + 1
        # determine background detected as outliers
        else:
            if distance_array[j] > threshold:
                background_outliers = background_outliers + 1

    # determine number of background
    background_count = popsize - ball_count

    tr = float(float(ball_outliers)/ float(ball_count))
    fr = float(background_outliers)/ float(background_count)

    print "t = ", threshold , " ball corners = ", ball_count , "tr = ", ball_outliers, "fr = ", background_outliers, "trate = ", tr, "frate = ", fr

    #append true rate and false rate
    true_rate.append(tr)
    false_rate.append(fr)

#plt.ion()



print "false_rate"
print false_rate
print "true_rate"
print true_rate

xdata = []
ydata = []
tdata = []
youndexvalue = 0
youndexindex = 0
mindistvalue = 100000
mindistindex = 0

for i in range(tmax-2, 1, -1):
    print false_rate[i], "," , true_rate[i], ",", threshold_array[i]
    xdata.append(false_rate[i])
    ydata.append(true_rate[i])
    tdata.append(threshold_array[i])
    j = false_rate[i]+ true_rate[i]
    if j > youndexvalue:
         youndexvalue = j
         youndexindex = i
    d = math.sqrt(false_rate[i] * false_rate[i] + (1 - true_rate[i]) * (1 - true_rate[i]))
    if d < mindistvalue:
         mindistvalue = d
         mindistindex = i

print "youndex index " , youndexindex, " threshold = ", threshold_array[youndexindex]
print "mindist index ", mindistindex, " threshold = " , threshold_array[mindistindex]

 

#xdata = false_rate
#ydata = true_rate
#tdata = range(1,tmax-1)

#xdata = [1,2,3,4,5]
#ydata = [1,1,2,2,3]
#tdata = [6,7,8,9,10]

def onpick3(event):
    #index = event.ind
    thisline = event.artist
    xdata1 = thisline.get_xdata()
    ydata1 = thisline.get_ydata()
    index = 0
    while index < 5:
        if xdata[index] == xdata1[0] and ydata[index] == ydata1[0]:
            break
        index = index + 1
    print "index " , index
    print 'onpick3 point:', index, xdata1[0], ydata1[0], tdata[index]
    #ax.plot(xdata[i], ydata[i], 'o', picker=5)
    ax.text(xdata1[0]+2, ydata1[0]+2, 't='+`tdata[index]`, fontsize=12)
    #plt.draw()
    plt.show()

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(xdata,ydata)
#for i in range(1,5):
    #ax.plot(xdata[i], ydata[i], 'o', picker=5)
    #ax.text(xdata[i], ydata[i], 't='+`tdata[i]`, fontsize=12)
#plot optimal threshold
ax.text(false_rate[mindistindex], true_rate[mindistindex], 't='+`threshold_array[mindistindex]`+' tr= ' + `true_rate[mindistindex]` + ' fr= ' + `false_rate[mindistindex]`, fontsize=12)

x1,x2,y1,y2 = plt.axis()
plt.axis((x1,x2,0,1.2))
#plt.axis((y1,y2,0,1))
#fig.savefig('pscoll.eps')
fig.canvas.mpl_connect('pick_event', onpick3)

show()
