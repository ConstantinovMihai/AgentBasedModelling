path =[(1, 6), (1, 6), (1, 6), (1, 5), (0, 5), (0, 4), (0, 3), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2), (0, 2)]

trim_length = 0
found = False
while found == False:
    if path[-trim_length-1] != path[-trim_length-2]:
        found = True
    else:
        trim_length += 1

path2 = path[:len(path)-trim_length]
print(path2)