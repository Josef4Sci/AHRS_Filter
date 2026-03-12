from dataclasses import dataclass


@dataclass
class OnePoleLP:
    """1st-order IIR low-pass: y += alpha * (x - y)"""
    alpha: float
    y: float = 0.0

    def update(self, x: float) -> float:
        self.y += self.alpha * (x - self.y)
        return self.y


@dataclass
class CascadeLP:
    """Cascade N identical 1-pole stages for steeper high-frequency attenuation."""
    alpha: float
    stages: int
    y: list = None

    def __post_init__(self):
        if self.stages < 1:
            raise ValueError("stages must be >= 1")
        if self.y is None:
            self.y = [0.0] * self.stages

    def update(self, x: float) -> float:
        v = x
        for i in range(self.stages):
            yi = self.y[i]
            yi += self.alpha * (v - yi)
            self.y[i] = yi
            v = yi
        return v

class LP1(OnePoleLP):
    def __init__(self, alpha: float, y0: float = 0.0):
        super().__init__(alpha=alpha, y=y0)

# Convenience wrappers
class LP2(CascadeLP):
    def __init__(self, alpha: float, y0: float = 0.0):
        super().__init__(alpha=alpha, stages=2, y=[y0, y0])

class LP4(CascadeLP):
    def __init__(self, alpha: float, y0: float = 0.0):
        super().__init__(alpha=alpha, stages=4, y=[y0, y0, y0, y0])

# if __name__ == "__main__":
#     # Example usage
#     lp2 = LP2(alpha=0.1, y0=0.5)
#     lp4 = LP4(alpha=0.1, y0=0.5)
#     inter = OnePoleLP(alpha=0.1, y=0.5)

#     samples = [0, 1, 0, 1, 0, 1]  # toy signal
#     print("LP2:", [lp2.update(x) for x in samples])
#     print("LP4:", [lp4.update(x) for x in samples])
#     print("inter:", [inter.update(x) for x in samples])
#     import matplotlib.pyplot as plt
#     plt.plot(samples, label='Input')
#     plt.plot([lp2.update(x) for x in samples], label='LP2')
#     plt.plot([lp4.update(x) for x in samples], label='LP4')
#     plt.plot([inter.update(x) for x in samples], label='OnePoleLP')
#     plt.legend()
#     plt.show()