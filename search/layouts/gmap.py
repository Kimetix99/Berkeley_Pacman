import sys
import random

class RandomMap:

	def __init__(self, nr, nc, prob, seed):
		self.nr = nr
		self.nc = nc
		self.prob = prob
		self.seed = seed

	def gen_map(self):
		
		nr = self.nr
		nc = self.nc
		prob = self.prob
		seed = self.seed
		random.seed(self.seed)

		pacman_pos = (random.randint(1,nr-2),random.randint(1,nc-2))
		food_pos = (random.randint(1,nr-2),random.randint(1,nc-2))
		while pacman_pos == food_pos:
			food_pos = (random.randint(1,nr-2),random.randint(1,nc-2))

		for i in range(nr):
			for j in range(nc):
				if i>0 and i<nr-1 and j>0 and j<nc-1:
					if pacman_pos == (i,j): sys.stdout.write('P')
					elif food_pos == (i,j): sys.stdout.write('.')
					elif random.random() < prob: sys.stdout.write('%')
					else: sys.stdout.write(' ')
				else: 
					sys.stdout.write('%')
			print ""

if __name__ == "__main__":

	nr = int(sys.argv[1])
	nc = int(sys.argv[2])
	prob = float(sys.argv[3])
	seed = int(sys.argv[4])

	rmap = RandomMap(nr,nc,prob,seed)
	rmap.gen_map()


