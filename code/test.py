path = [(1, 20), (2, 20), (3, 20), (4, 20), (5, 20), (6, 20), (7, 20), (8, 20), (8, 20), (8, 20), (8, 20),(7, 20), (8, 20), (8,20)]

i = 0
found = False
while found == False:
    if path[-i-1] != path[-i-2]:
        found = True
    else:
        i += 1

print(i)

path = path[:len(path)-i]
print(path)
