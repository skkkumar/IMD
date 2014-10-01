import yarp
import matplotlib.pyplot as plt
import time
plt.ion()
yarp.Network.init()
data1 = []
data2 = []
threshold = []
fig = plt.figure()
#p = yarp.BufferedPortImageRgb()
p1 = yarp.BufferedPortBottle()
#p.open("/roc:o");
p1.open("/roc:i");
yarp.Network.connect("/roc:o", "/roc:i")
top = 1;
first = 1;
while(top):
    bin = yarp.Bottle()
    bin = p1.read(True)
    while top < bin.size()-3 and first:
      if (bin):
	#print("Received [%s]"%bin.toString())
	data1 = data1 + [bin.get(top-1).asDouble()]
	top = top + 1
	data2 = data2 + [bin.get(top-1).asDouble()]
	top = top + 1
	threshold = threshold + [bin.get(top - 1).asInt()]
      elif not(bin):
	print("Failed to read input")
      top = top + 1
    first = 0
    if (bin):
      #print("Received [%s]"%bin.toString())
      data1 = data1 + [bin.get(top-1).asDouble()]
      top = top + 1
      data2 = data2 + [bin.get(top-1).asDouble()]
      top = top + 1
      threshold = threshold + [bin.get(top - 1).asInt()]
      #plt.close("all")
      #fig = plt.figure()
      plt.close("all")
      plt.figure()
    elif not(bin):
      print("Failed to read input")
      #return False
      #plt.close()
    plt.plot(data2, data1)
    
    plt.ylabel('True Positive Rate')
    
    plt.xlabel('False Positive Rate')
    for i,j,t in zip(data1,data2,threshold):
      plt.annotate(str(t),xy = (j,i))
    plt.draw()
    #time.sleep(5)
    #img = p.prepare()
    #img.resize(320,240)
    #img.zero()
    #img.pixel(160,120).r = 255
    #p.write()
    yarp.Time.delay(0.5)
    top = top + 1
p1.close();
yarp.Network.fini();


#import yarp

#yarp.Network.init()

#class DataProcessor(yarp.PortReader):
    #def read(self,connection):
        #print("in DataProcessor.read")
        #if not(connection.isValid()):
            #print("Connection shutting down")
            #return False
        #bin = yarp.Bottle()
        #bout = yarp.Bottle()
        #print("Trying to read from connection")
        #ok = bin.read(connection)
        #if (ok):
	  #print("Received [%s]"%bin.toString())
	  #data = data + bin.toInt()
        #elif not(ok):
            #print("Failed to read input")
            #return False
        #bout.append()
        #bout.append(bin)
        
        
	
#plt.ylabel('some numbers')
#plt.show()
        #print("Sending [%s]"%bout.toString())
        #writer = connection.getWriter()
        #if writer==None:
            #print("No one to reply to")
            #return True
        #return bout.write(writer)
#data = []
#p = yarp.BufferedPortBottle()
#r = DataProcessor()
#p.setReader(r)
#p.open("/roc:i");

#yarp.Time.delay(100)
#print("Test program timer finished")

#yarp.Network.fini();