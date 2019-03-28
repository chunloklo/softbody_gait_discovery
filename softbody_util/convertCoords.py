import csv
import sys

if (len(sys.argv) < 3):
    print("Usage: python convertCoords.py infile.txt outfile.txt")
    quit()


# print(sys.argv[1])


# print(sys.argv[2])

def recoverCoord(x, size):
    x = float(x)
    size = float(size)
    centerOffset = size / 2
    return int((x - centerOffset) / size)

# print(recoverCoord(0.0015, 0.001))



size = 1
zOffset = 0
voxels = []

with open(sys.argv[1]) as tsv:
    heading = None
    reader = csv.reader(tsv, delimiter='\t') #You can also use delimiter="\t" rather than giving a dialect.
    if(heading == None):
        next(reader, None);
    for line in reader:
        matIndex, x, y, z = line
        matIndex = int(matIndex) - 1

        x = recoverCoord(x, size)
        y = recoverCoord(y, size)
        z = recoverCoord(z, size)

        print(x, y, z, matIndex)
        voxels.append([x, y, z, matIndex])
print(voxels)


with open(sys.argv[2], "w+") as output:
    output.write("[")
    voxels = ["{}, {}, {}, {}".format(v[0], v[1], v[2] + zOffset, v[3]) for v in voxels]
    outstring = ",\n".join(voxels)
    output.write(outstring)
    output.write("]")