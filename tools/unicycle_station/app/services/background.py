import threading
from PySide6 import QtCore


class JobSignals(QtCore.QObject):
    finished = QtCore.Signal(object)
    failed = QtCore.Signal(str)
    progress = QtCore.Signal(int)


class BackgroundJob(QtCore.QRunnable):
    """Disk-heavy work off the GUI thread; cooperative cancellation."""
    def __init__(self, function):
        super().__init__()
        self.function = function
        self.signals = JobSignals()
        self.cancelled = threading.Event()

    def report(self, value):
        if self.cancelled.is_set():
            raise InterruptedError("操作已取消")
        self.signals.progress.emit(value)

    def run(self):
        try:
            result = self.function(self.report)
            if self.cancelled.is_set():raise InterruptedError("操作已取消")
            self.signals.finished.emit(result)
        except Exception as exc:
            self.signals.failed.emit(str(exc))
