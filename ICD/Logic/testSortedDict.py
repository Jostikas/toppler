from sortedcollections import ValueSortedDict

b = {1:(1,6), 2:(2,5), 3:(3,4), 4:(4,3), 5:(5,2), 6:(6,1)}
a = ValueSortedDict(lambda x: x[1])
a.update(b)

print(a)
# a.update(((3, (3, 9)),))
a[3] = (3, 9)
print(a)
print(list(a.irange(4, 2)))
print(list(a.irange_key(2,4)))
