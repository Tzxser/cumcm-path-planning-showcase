from Q3.engine import Config, Solver
def build():return Solver(Config("Q3_X13",routing="tour",stop_at_16_discovered=True,service_probe_cap=3))
