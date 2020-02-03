import multiprocessing as mp
import time
import random


def cube(x):
	return x**3

if __name__ == '__main__':
	
	pool = mp.Pool(processes=4)

	arr = random.sample(range(1000), 100)

	start = time.time()
	res_simple = []
	for i in arr:
		res_simple.append(cube(i))
	end = time.time()
	print(f'simple loop took {end - start} secs')

	start_async = time.time()
	handles = [pool.apply_async(cube, args=(i, )) for i in arr]
	res = [h.get() for h in handles]
	end_async = time.time()
	print(f'async loop took {end_async - start_async} secs')
