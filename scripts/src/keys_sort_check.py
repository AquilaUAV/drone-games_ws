keys = [-2, -1, 0, 1, 2, 3, 4, 5, 6]
keys.sort()
if len(keys) == 0:
    exit()
keys_neg = [key for key in keys if key < 0]
keys = [key for key in keys if key >= 0]
keys.extend(keys_neg[::-1])
print(keys)
