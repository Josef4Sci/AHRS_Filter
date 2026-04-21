from unittest import result
import numpy as np

# import the algorithm from the uber_turbo package

from turbo import  Turbo1, TurboM

class TuRBO_BO:
    """
    Wrapper around the official uber-turbo (TuRBO) implementation for minimization.
    """

    def __init__(
        self,
        f,
        pbounds: dict,
        n_init: int = 20,
        batch_size: int = 5,
        use_multi: bool = False,
        num_tr: int = 5,
        datasets=None
    ):
        """
        f           : function to minimize
        pbounds     : dict of parameter bounds e.g. {"x": (0,1), "y": (-5,5)}
        n_init      : number of initial random points per trust region
        batch_size  : number of points per iteration
        use_multi   : if True, use TurboM (multiple trust regions)
        num_tr      : number of trust regions for TurboM
        random_state: random seed
        """
        self.f = f
        self.pbounds = pbounds
        self.dim = len(pbounds)
        self.n_init = n_init
        self.batch_size = batch_size
        self.use_multi = use_multi
        self.num_tr = num_tr
        self.datasets = datasets

        # convert bounds into arrays
        if type(pbounds['low_band'])==float:
            self.lb = np.array([pbounds['low_band']])
            self.ub = np.array([pbounds['up_band']])
        else:
            self.lb = np.array(pbounds['low_band'])
            self.ub = np.array(pbounds['up_band'])
            
    def run(self, n_iter: int = 50):
        """
        Run the TuRBO optimization for n_iter rounds.
        Returns best point and best value (minimization).
        """

        # choose the TuRBO variant
        if self.use_multi:
            optimizer = TurboM(
                f=self.new_f,
                lb=self.lb,
                ub=self.ub,
                n_init=self.n_init,
                max_evals=n_iter * self.batch_size + self.n_init,
                batch_size=self.batch_size
            )
        else:
            optimizer = Turbo1(
                f=self.new_f,
                lb=self.lb,
                ub=self.ub,
                max_evals=n_iter * self.batch_size + self.n_init,
                n_init=self.n_init,
                batch_size=self.batch_size
            )

        # run the optimization
        optimizer.optimize()
    

        X = optimizer.X  # Evaluated points
        fX = optimizer.fX  # Observed values
        ind_best = np.argmin(fX)
        f_best, x_best = fX[ind_best], X[ind_best, :]
        
        return x_best, f_best
    
    def new_f(self, x):
        return self.f(x, self.datasets)