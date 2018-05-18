import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

nx = 70
ny = 70

rawData = open("rocketPod.txt", 'r')
dataLines = rawData.readlines()
    
fig = plt.figure()
pixels = np.zeros((64))

HDTemp = np.zeros((80,80))
data = np.uint8(np.zeros((nx, ny, 3)))
im = plt.imshow(data)

# start with some initial colors
MinTemp = 25
MaxTemp = 35
a = MinTemp + (MaxTemp - MinTemp) * 0.2121
b = MinTemp + (MaxTemp - MinTemp) * 0.3182
c = MinTemp + (MaxTemp - MinTemp) * 0.4242
d = MinTemp + (MaxTemp - MinTemp) * 0.8182


num_lines = sum(1 for line in open('rocketPod.txt'))
#num_lines = 1

print('num_lines: {}' .format(num_lines))
def init():
    im.set_data(np.uint8(np.zeros((nx, ny, 3))))

# function to get the cutoff points in the temp vs RGB graph
def Getabcd():
  a = MinTemp + (MaxTemp - MinTemp) * 0.2121
  b = MinTemp + (MaxTemp - MinTemp) * 0.3182
  c = MinTemp + (MaxTemp - MinTemp) * 0.4242
  d = MinTemp + (MaxTemp - MinTemp) * 0.8182

   
    
def constrain(val, min_val, max_val):

    if val < min_val: 
        val = min_val

    elif val > max_val: 
        val = max_val

    return val    
    
#color interpolation routine
def GetColor(val) :

  red = constrain(255.0 / (c - b) * val - ((b * 255.0) / (c - b)), 0, 255)

  if ((val > MinTemp) & (val < a)) :
    green = constrain(255.0 / (a - MinTemp) * val - (255.0 * MinTemp) / (a - MinTemp), 0, 255)
  elif ((val >= a) & (val <= c))  :
    green = 255
  elif (val > c) :
    green = constrain(255.0 / (c - d) * val - (d * 255.0) / (c - d), 0, 255)
  elif ((val > d) | (val < a)) : 
    green = 0
  
  if (val <= b) :
    blue = constrain(255.0 / (a - b) * val - (255.0 * b) / (a - b), 0, 255) 
  elif ((val > b) & (val <= d)) : 
    blue = 0 
  elif (val > d) :
    blue = constrain(240.0 / (MaxTemp - d) * val - (d * 240.0) / (MaxTemp - d), 0, 240)
  # use the displays color mapping function to get 5-6-5 color palet (R=5 bits, G=6 bits, B-5 bits)
  return (red, green, blue)

    
    
# interplation function to create 70 columns for 8 rows
def InterpolateRows():

  # interpolate the 8 rows (interpolate the 70 column points between the 8 sensor pixels first)
  for row in range (0,8):
    for col in range (0,70):
      # get the first array point, then the next
      # also need to bump by 8 for the subsequent rows
      aLow =  int(col / 10 + (row * 8))
      aHigh = int((col / 10) + 1 + (row * 8))
      # get the amount to interpolate for each of the 10 columns
      # here were doing simple linear interpolation mainly to keep performace high and
      # display is 5-6-5 color palet so fancy interpolation will get lost in low color depth
      intPoint =   (( pixels[aHigh] - pixels[aLow] ) / 10.0 )
      #print ('{} {} {}' .format(intPoint, pixels[aHigh], pixels[aLow]), end = ' ')
      # determine how much to bump each column (basically 0-9)
      incr = col % 10
      # find the interpolated value
      val = (intPoint * incr ) +  pixels[aLow]
      # store in the 70 x 70 array
      # since display is pointing away, reverse row to transpose row data
      #print(' {} [{},{}]' .format(val,((7 - row) * 10), col))
      HDTemp[((7 - row) * 10), col] = val

   
#interplation function to interpolate 70 columns from the interpolated rows
def InterpolateCols() :

  # then interpolate the 70 rows between the 8 sensor points
  for col in range (0,70) :
    for row in range (0,70):
      # get the first array point, then the next
      # also need to bump by 8 for the subsequent cols
      aLow =  int((row / 10 )) * 10
      aHigh = int(aLow + 10)
      # get the amount to interpolate for each of the 10 columns
      # here were doing simple linear interpolation mainly to keep performace high and
      # display is 5-6-5 color palet so fancy interpolation will get lost in low color depth
      intPoint =   (( HDTemp[aHigh,col] - HDTemp[aLow,col] ) / 10.0 )
      #print ('{} {} {}' .format(intPoint, HDTemp[aHigh,col], HDTemp[aLow,col]), end = ' ')
      # determine how much to bump each column (basically 0-9)
      incr = row % 10
      # find the interpolated value
      val = (intPoint * incr ) +  HDTemp[aLow,col]
      # store in the 70 x 70 array
      #print(' {} [{},{}][{},{}]->[{},{}]' .format(val,aHigh,col,aLow,col,row,col))
      HDTemp[row ,col] = val
    #print ('')
  
    
    
def animate(i): #this i is going to have to be line indexes
# color calc and array pop will need to happen here.

    print ("Printing data line: {}" .format(i))
    if(i!=0):
        for j in range (0, 64):
            pixels[j] = float(dataLines[i][(j*6):((j*6)+5)])
            print('{} ' .format(dataLines[i][(j*6):((j*6)+5)]), end = ' ')
        InterpolateRows()
        InterpolateCols()
        for j in range(0,nx):
            for x in range(0,ny):
                #dataRed, dataGreen, dataBlue = GetColor(pixels[((8*j)+x)]) #8x8 array
                #print(pixels[((8*j)+x)], end = ' ')
                #print(HDTemp[j, x], end = ' ')
                dataRed, dataGreen, dataBlue = GetColor(HDTemp[j, x])
                data[x,j] = np.array([dataRed,dataGreen,dataBlue])
    im.set_data(data)
    return im

anim = animation.FuncAnimation(fig, animate, init_func=init, frames=num_lines, interval=200, repeat=True)
plt.colorbar()
plt.show()