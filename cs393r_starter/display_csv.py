from numpy import genfromtxt
from PIL import Image

my_data = genfromtxt('image3.csv', delimiter=',')
# my_data = my_data * -100
for j in my_data:
    for i in range(len(j)):
        if j[i] < 0:
            j[i] = 4 / (-0.01 * j[i]) 
im = Image.fromarray(my_data)
if im.mode != 'RGB':
    im = im.convert('RGB')
im.save("your_file3.jpeg")
