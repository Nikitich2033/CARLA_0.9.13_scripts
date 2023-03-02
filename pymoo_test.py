from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.factory import get_problem
from pymoo.optimize import minimize

# define the problem
problem = get_problem("zdt1")

# define the optimization algorithm
algorithm = NSGA2()

# run the optimization
res = minimize(problem,
               algorithm,
               ('n_gen', 100),
               seed=1,
               verbose=False)

# print the final Pareto front
print(res.F)
