import time
from functools import wraps


def timeit(f):
    @wraps(f)
    def wrap(*args, **kw):
        tstart = time.time()
        result = f(*args, **kw)
        tend = time.time()
        print(f"[{f.__name__}]: {tend - tstart} sec")
        return result
    return wrap
