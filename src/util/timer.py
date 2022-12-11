import time
import logging


def timeit(f):  # decorator

    def timed(*args, **kw):
        ts = time.time()
        class_name, method_name = f.__qualname__.split('.')
        result = f(**args, **kw)
        te = time.time()
        method = '.'.join(class_name, method_name)
        logging.critical(f'...\'{method}\' took {te-ts:2.4f} sec')
        return result
