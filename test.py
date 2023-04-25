import numpy as np

# l = []
arr = [[43, 30], [36, 37], [34, 6], [47, 1], [42, 18], [43, 49], [1, 36], [40, 11], [47, 35], [46, 12], [42, 42], [35, 35]]
# arr = [a for a in arr if a != [8,9,10]]


# def collisions_calc(collisions):
#     if collisions != None:
#         flat_list = flatten_list(collisions)
#     collisions_augmented = [[flat_list[i], flat_list[i+1]] for i in range(0, len(flat_list), 2)]
#     return collisions_augmented

# def flatten_list(nested_list):
#     flat_list = []
#     for item in nested_list:
#         if isinstance(item, list):
#             flat_list.extend(flatten_list(item))
#         else:
#             flat_list.append(item)
#     return flat_list


# flat_list = flatten_list(arr)
# print(flat_list)

# pairs_list = [[flat_list[i], flat_list[i+1]] for i in range(0, len(flat_list), 2)]
# print(pairs_list)

print(arr.index([34,6]))