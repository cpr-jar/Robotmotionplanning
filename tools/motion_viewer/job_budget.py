"""One editor job's monotonic deadline and cooperative cancellation.

The event may be set by an HTTP thread; processing stays on its single worker.
Native subprocesses are terminated by that worker, never by the request thread.
"""

import math
import threading
import time


class JobStopped(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class JobBudget:
    def __init__(self, timeout_s):
        if not math.isfinite(timeout_s) or not 0 < timeout_s <= 3600:
            raise ValueError("timeout_s must be positive and at most 3600")
        self.deadline = time.monotonic() + timeout_s
        self.cancelled = threading.Event()

    def checkpoint(self):
        if self.cancelled.is_set():
            raise JobStopped("cancelled", "Simulation abgebrochen; keine neue Ansicht freigegeben.")
        if time.monotonic() >= self.deadline:
            raise JobStopped("deadline_exceeded", "Zeitbudget überschritten; Simulation abgebrochen.")

    def remaining(self):
        self.checkpoint()
        return max(0.001, self.deadline - time.monotonic())
