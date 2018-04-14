import matplotlib.pyplot as pyplot
import matplotlib.animation as animation
import itertools

fig = pyplot.figure()
baro = fig.add_subplot(111)

def refreshBaroData(i):
    graphData = open("baro_data.txt", "r").read()
    lines = graphData.split("\n")
    xValues = []
    yValues = []
    for line in lines:
        if len(line) > 1:
            x, y = line.split(",")
            xValues.append(float(x))
            yValues.append(float(y) - 394)

    try:
        lists = sorted(itertools.izip(*[xValues,yValues]))
        new_x, new_y = list(itertools.izip(*lists))
        print(new_y)
        baro.clear()
        baro.plot(new_x, new_y)
    except:
        pass


ani = animation.FuncAnimation(fig, refreshBaroData, interval = 1000)
pyplot.show()
